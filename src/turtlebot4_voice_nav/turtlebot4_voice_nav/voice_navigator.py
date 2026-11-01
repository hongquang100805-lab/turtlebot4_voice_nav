"""Validated Vietnamese routes -> one owned NavigateToPose action at a time."""
import json
import math
from pathlib import Path
import signal
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.msg import DockStatus
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .command_parser import parse_command
from .map_regions import load_regions


def default_regions():
    return str(Path.home() / 'tb4_ws/src/turtlebot4_voice_nav/config/generated/regions.yaml')


class VoiceNavigator(Node):
    """Reject unknown destinations atomically; busy commands never replace a route."""

    def __init__(self):
        super().__init__('voice_navigator')
        for key, value in {'regions_file': default_regions(), 'goal_timeout': 600.0,
                           'response_timeout': 15.0, 'transform_max_age': 2.0}.items():
            self.declare_parameter(key, value)
        for key in ('goal_timeout', 'response_timeout', 'transform_max_age'):
            value = float(self.get_parameter(key).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be positive and finite')
        self.regions_data = load_regions(self.get_parameter('regions_file').value)
        self.regions = {r['id']: r for r in self.regions_data['regions']}
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/voice/navigation_status', durable)
        self.marker_pub = self.create_publisher(MarkerArray, '/voice/route_markers', durable)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = {name: self.create_client(GetState, f'/{name}/get_state')
                          for name in ('amcl', 'bt_navigator')}
        self.active = {}; self.pending = {}; self.last_query = 0.0
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.amcl = False; self.docked = None; self.dock_at = 0.0
        self.phase = 'IDLE'; self.route = []; self.index = 0; self.ready = False
        self.goal_handle = self.send_future = self.result_future = None
        self.cancel_requested = False; self.cancel_at = 0.0; self.cancel_warned = False
        self.clock_ns = 0; self.clock_at = time.monotonic(); self.unready_at = None
        self.sent_at = 0.0; self.accepted_at = 0.0; self.last_feedback = 0.0
        self.create_subscription(String, '/voice/transcript', self.on_text, 1)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.on_amcl, durable)
        self.create_subscription(DockStatus, '/dock_status', self.on_dock,
                                 qos_profile_sensor_data)
        self.create_service(Trigger, '/voice/cancel', self.cancel_service)
        self.create_timer(0.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.report('STARTED', 'Đợi AMCL, Nav2, TF và robot rời dock.')

    @property
    def busy(self):
        return self.send_future is not None or self.goal_handle is not None

    def report(self, event, message, **extra):
        data = {'state': self.phase, 'event': event, 'message': message,
                'ready': self.ready, 'route': self.route, 'waypoint_index': self.index,
                'active_region': self.route[self.index] if self.busy and self.index < len(self.route) else None,
                'wall_time': time.time(), **extra}
        self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))
        self.get_logger().info(json.dumps(data, ensure_ascii=False))

    def on_amcl(self, msg):
        self.amcl = msg.header.frame_id == 'map'

    def on_dock(self, msg):
        self.docked = msg.is_docked; self.dock_at = time.monotonic()

    def on_text(self, msg):
        try:
            command = parse_command(msg.data, self.regions_data)
        except ValueError as exc:
            self.report('REJECTED', str(exc), text=msg.data)
            return
        if command['intent'] == 'cancel':
            self.cancel('Yêu cầu dừng bằng giọng nói.'); return
        if self.busy:
            self.report('BUSY', 'Đang có hành trình; dừng/hủy trước khi gửi lệnh mới.', text=msg.data)
            return
        if not self.ready:
            self.report('NOT_READY', 'Chưa sẵn sàng: kiểm tra Play, AMCL, Nav2, TF và undock.', text=msg.data)
            return
        self.route = list(command['region_ids']); self.index = 0
        self.cancel_requested = False
        self.publish_route()
        self.report('ROUTE_ACCEPTED', 'Đã kiểm tra toàn bộ các khu vực.', text=msg.data)
        self.dispatch()

    def publish_route(self):
        out = MarkerArray(); clear = Marker(); clear.action = Marker.DELETEALL
        out.markers.append(clear)
        for i, region_id in enumerate(self.route):
            goal = self.regions[region_id]['goal']
            marker = Marker(); marker.header.frame_id = 'map'
            marker.ns = 'voice_route'; marker.id = i; marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD; marker.pose.position.x = float(goal['x'])
            marker.pose.position.y = float(goal['y']); marker.pose.position.z = .6
            marker.pose.orientation.w = 1.0; marker.scale.z = .55
            marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
            marker.text = f'{i + 1}: {region_id}'
            out.markers.append(marker)
        self.marker_pub.publish(out)

    def dispatch(self):
        if not self.ready:
            self.phase = 'FAILED'; self.report('NOT_READY', 'Dừng chuỗi: localization/Nav2 chưa sẵn sàng.')
            return
        goal = self.regions[self.route[self.index]]['goal']
        request = NavigateToPose.Goal(); request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = float(goal['x']); request.pose.pose.position.y = float(goal['y'])
        request.pose.pose.orientation.z = math.sin(float(goal['yaw']) / 2)
        request.pose.pose.orientation.w = math.cos(float(goal['yaw']) / 2)
        self.phase = 'SENDING'; self.sent_at = time.monotonic()
        try:
            self.send_future = self.client.send_goal_async(request, feedback_callback=self.on_feedback)
            self.send_future.add_done_callback(self.on_accepted)
            self.report('GOAL_SENT', 'Gửi đích đến Nav2.', goal=goal)
        except Exception as exc:
            # A transport error cannot prove whether the server received a request.
            self.send_future = Future()
            self.cancel_requested = True; self.cancel_at = time.monotonic()
            self.phase = 'ERROR'; self.report('SEND_UNKNOWN', f'Giữ khóa hành trình: {exc}')

    def on_accepted(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.cancel_requested = True; self.cancel_at = time.monotonic()
            self.phase = 'ERROR'
            self.report('SEND_UNKNOWN', f'Chưa xác nhận request tới server; giữ khóa hành trình: {exc}'); return
        self.send_future = None
        if not self.goal_handle or not self.goal_handle.accepted:
            self.goal_handle = None; self.phase = 'FAILED'
            self.report('GOAL_REJECTED', 'Nav2 từ chối goal; không đi tới điểm tiếp theo.'); return
        self.accepted_at = time.monotonic()
        try:
            self.result_future = self.goal_handle.get_result_async()
            self.result_future.add_done_callback(self.on_result)
        except Exception as exc:
            self.cancel_requested = True; self.phase = 'ERROR'
            self.report('RESULT_UNKNOWN', str(exc)); self.request_cancel(); return
        if self.cancel_requested:
            self.request_cancel()
        else:
            self.phase = 'NAVIGATING'; self.report('GOAL_ACCEPTED', 'Nav2 đang điều hướng.')

    def on_result(self, future):
        try:
            response = future.result()
        except Exception as exc:
            # Keep ownership locked: transport error does not prove the robot stopped.
            self.phase = 'ERROR'; self.report('RESULT_UNKNOWN', f'Chưa xác nhận kết thúc action: {exc}')
            self.cancel_requested = True; self.request_cancel(); return
        self.goal_handle = self.result_future = self.send_future = None
        error_code = int(getattr(response.result, 'error_code', 0))
        if self.cancel_requested:
            self.phase = 'CANCELED'; self.report('ROUTE_CANCELED', 'Action đã kết thúc; chuỗi bị hủy.', action_status=response.status)
            self.cancel_requested = False; return
        if response.status != GoalStatus.STATUS_SUCCEEDED or error_code != 0:
            self.phase = 'FAILED'
            self.report('GOAL_FAILED', 'Dừng chuỗi tại goal không thành công.', action_status=response.status,
                        error_code=error_code, error_msg=getattr(response.result, 'error_msg', ''))
            return
        self.report('WAYPOINT_REACHED', f'Đã đến khu vực {self.route[self.index]}.')
        self.index += 1
        if self.index == len(self.route):
            self.phase = 'COMPLETED'; self.report('ROUTE_COMPLETED', 'Đã hoàn thành toàn bộ hành trình.')
        else:
            self.dispatch()

    def on_feedback(self, feedback):
        if time.monotonic() - self.last_feedback > 2.0:
            self.report('FEEDBACK', 'Đang di chuyển.', distance_remaining=float(feedback.feedback.distance_remaining))
            self.last_feedback = time.monotonic()

    def cancel_service(self, request, response):
        del request
        response.success = self.cancel('Yêu cầu hủy qua service.')
        response.message = 'Đã yêu cầu hủy; chờ ROUTE_CANCELED.' if response.success else 'Không có action của node này đang chạy.'
        return response

    def cancel(self, reason):
        if not self.busy:
            self.report('NO_ACTIVE_ROUTE', 'Không có hành trình đang chạy.'); return False
        if self.cancel_requested:
            return True
        self.cancel_requested = True; self.phase = 'CANCELING'; self.cancel_at = time.monotonic()
        self.report('CANCEL_REQUESTED', reason)
        if self.goal_handle is not None:
            self.request_cancel()
        # A late send response is canceled in on_accepted, never leaked as a new goal.
        return True

    def request_cancel(self):
        if self.goal_handle is None:
            return
        self.phase = 'CANCELING'; self.cancel_at = time.monotonic()
        try:
            self.goal_handle.cancel_goal_async().add_done_callback(self.on_cancel_response)
        except Exception as exc:
            self.report('CANCEL_NOT_CONFIRMED', f'Giữ khóa hành trình: {exc}')

    def on_cancel_response(self, future):
        try:
            response = future.result()
            if not response.goals_canceling and self.busy:
                self.report('CANCEL_NOT_CONFIRMED', 'Chưa được server xác nhận hủy; giữ khóa hành trình.')
        except Exception as exc:
            self.report('CANCEL_NOT_CONFIRMED', str(exc))

    def check_ready(self, now):
        stamp = self.get_clock().now().nanoseconds
        if stamp != self.clock_ns:
            if stamp < self.clock_ns:
                self.amcl = False
            self.clock_ns = stamp; self.clock_at = now
        for name, future in list(self.pending.items()):
            if future.done():
                try:
                    self.active[name] = (future.result().current_state.id == State.PRIMARY_STATE_ACTIVE, now)
                except Exception:
                    self.active[name] = (False, now)
                del self.pending[name]
        if now - self.last_query > 1.0:
            for name, client in self.lifecycle.items():
                if name not in self.pending and client.service_is_ready():
                    self.pending[name] = client.call_async(GetState.Request())
            self.last_query = now
        ready = (stamp > 0 and now - self.clock_at < 2.0 and self.amcl
                 and self.docked is False and now - self.dock_at < 5.0
                 and all(self.active.get(name, (False, 0))[0]
                         and now - self.active.get(name, (False, 0))[1] < 5.0 for name in self.lifecycle)
                 and self.client.server_is_ready())
        if ready:
            try:
                tf = self.buffer.lookup_transform('map', 'base_link', Time())
                age = (stamp - Time.from_msg(tf.header.stamp).nanoseconds) / 1e9
                ready = -.2 <= age <= float(self.get_parameter('transform_max_age').value)
            except TransformException:
                ready = False
        return ready

    def tick(self):
        now = time.monotonic(); ready = self.check_ready(now)
        if ready != self.ready:
            self.ready = ready
            self.report('READY_CHANGED', 'Sẵn sàng nhận lệnh.' if ready else 'Chưa sẵn sàng điều hướng.')
        if not self.busy:
            self.unready_at = None; self.cancel_warned = False; return
        if not ready:
            if self.unready_at is None:
                self.unready_at = now
            elif now - self.unready_at > 3.0:
                self.cancel('Mất trạng thái sẵn sàng quá 3 giây.')
        else:
            self.unready_at = None
        if not self.cancel_requested:
            if self.phase == 'SENDING' and now - self.sent_at > self.get_parameter('response_timeout').value:
                self.cancel('Quá thời gian chờ chấp nhận; hủy ngay khi server trả lời.')
            elif self.phase == 'NAVIGATING' and now - self.accepted_at > self.get_parameter('goal_timeout').value:
                self.cancel('Goal quá thời gian thực cho phép.')
        elif self.cancel_at and now - self.cancel_at > 10 and not self.cancel_warned:
            self.cancel_warned = True
            self.report('CANCEL_WAITING', 'Chưa có kết quả cuối từ Nav2; chưa nhận hành trình mới.')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None; stopped = False
    def stop(signum, frame):
        nonlocal stopped
        stopped = True
    old = {s: signal.signal(s, stop) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        node = VoiceNavigator()
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if node is not None:
            node.cancel('Kết thúc node; hủy action đang sở hữu.')
            deadline = time.monotonic() + 8
            while node.busy and rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if node.busy:
                node.get_logger().error('Chưa xác nhận action dừng trước khi thoát; kiểm tra Nav2.')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in old.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
