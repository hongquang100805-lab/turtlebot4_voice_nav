"""Exercise the action protocol with real rclpy Futures, without a moving robot."""
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import String

from turtlebot4_voice_nav.voice_navigator import VoiceNavigator


class FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result_future = Future()
        self.cancel_future = Future()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future

    def finish(self, status=GoalStatus.STATUS_SUCCEEDED, error_code=0):
        self.result_future.set_result(SimpleNamespace(
            status=status, result=SimpleNamespace(error_code=error_code, error_msg='')))


class FakeClient:
    def __init__(self):
        self.requests = []
        self.responses = []

    def send_goal_async(self, request, feedback_callback=None):
        self.requests.append(request)
        future = Future()
        self.responses.append(future)
        return future


@pytest.fixture
def navigator():
    # Use the production callback methods and native Futures/messages, while
    # replacing external communication and time sources with deterministic peers.
    n = VoiceNavigator.__new__(VoiceNavigator)
    n.regions_data = {'regions': [
        {'id': 'A', 'name': 'Khu vực A', 'aliases': ['a'],
         'goal': {'x': 1.0, 'y': 2.0, 'yaw': 0.0}},
        {'id': 'B', 'name': 'Khu vực B', 'aliases': ['b'],
         'goal': {'x': 4.0, 'y': 5.0, 'yaw': 1.0}},
    ]}
    n.regions = {region['id']: region for region in n.regions_data['regions']}
    n.phase = 'IDLE'
    n.route = []
    n.index = 0
    n.ready = True
    n.goal_handle = n.send_future = n.result_future = None
    n.cancel_requested = False
    n.cancel_at = 0.0
    n.cancel_warned = False
    n.sent_at = n.accepted_at = n.last_feedback = 0.0
    n.unready_at = None
    n.client = FakeClient()
    n.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=10.0))
    n.events = []
    n.report = lambda event, message, **extra: n.events.append((event, extra))
    n.marker_updates = []
    n.publish_route = lambda: n.marker_updates.append(list(n.route))
    return n


def accept(n, request_index=0, accepted=True):
    handle = FakeHandle(accepted=accepted)
    n.client.responses[request_index].set_result(handle)
    return handle


def command(n, text='Di chuyển từ khu vực A đến khu vực B'):
    n.on_text(String(data=text))


def test_from_a_to_b_visits_a_first_and_advances_only_on_success(navigator):
    n = navigator
    command(n)
    assert n.route == ['A', 'B']
    assert len(n.client.requests) == 1
    assert n.client.requests[0].pose.header.frame_id == 'map'
    assert n.client.requests[0].pose.pose.position.x == 1.0
    first = accept(n)
    assert n.phase == 'NAVIGATING'
    assert len(n.client.requests) == 1
    first.finish()
    assert len(n.client.requests) == 2
    assert n.client.requests[1].pose.pose.position.x == 4.0
    second = accept(n, 1)
    second.finish()
    assert not n.busy
    assert n.phase == 'COMPLETED'
    assert n.events[-1][0] == 'ROUTE_COMPLETED'


@pytest.mark.parametrize('status,error_code', [
    (GoalStatus.STATUS_ABORTED, 0),
    (GoalStatus.STATUS_CANCELED, 0),
    (GoalStatus.STATUS_SUCCEEDED, 7),
])
def test_failure_or_nav2_error_stops_route(navigator, status, error_code):
    n = navigator
    command(n)
    handle = accept(n)
    handle.finish(status, error_code)
    assert len(n.client.requests) == 1
    assert n.index == 0
    assert n.phase == 'FAILED'
    assert not n.busy


def test_goal_rejection_stops_route(navigator):
    n = navigator
    command(n)
    accept(n, accepted=False)
    assert n.phase == 'FAILED'
    assert not n.busy
    assert len(n.client.requests) == 1


def test_unknown_destination_rejects_entire_route_before_sending(navigator):
    n = navigator
    command(n, 'Di chuyển từ khu vực A đến khu vực Z')
    assert not n.client.requests
    assert not n.marker_updates
    assert n.events[-1][0] == 'REJECTED'
    assert n.route == []


def test_busy_route_cannot_be_overwritten(navigator):
    n = navigator
    command(n)
    command(n, 'Đi đến khu vực B')
    assert n.route == ['A', 'B']
    assert len(n.client.requests) == 1
    assert n.events[-1][0] == 'BUSY'


def test_unknown_command_does_not_mutate_active_route(navigator):
    n = navigator
    command(n)
    command(n, 'Đi từ A đến khu vực Z')
    assert n.route == ['A', 'B']
    assert len(n.client.requests) == 1
    assert n.busy
    assert n.events[-1][0] == 'REJECTED'


def test_not_ready_rejects_without_queuing(navigator):
    n = navigator
    n.ready = False
    command(n)
    assert not n.client.requests
    assert not n.route
    assert n.events[-1][0] == 'NOT_READY'


def test_cancel_while_sending_cancels_late_acceptance(navigator):
    n = navigator
    command(n)
    command(n, 'Dừng lại')
    assert n.cancel_requested
    assert n.phase == 'CANCELING'
    handle = accept(n)
    assert handle.cancel_calls == 1
    assert n.busy
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
    # An ACK is not a terminal action result: replacement remains forbidden.
    command(n, 'Đi đến khu vực B')
    assert n.events[-1][0] == 'BUSY'
    assert len(n.client.requests) == 1
    handle.finish(GoalStatus.STATUS_CANCELED)
    assert n.phase == 'CANCELED'
    assert not n.busy
    assert len(n.client.requests) == 1


def test_cancel_racing_with_success_does_not_send_next_goal(navigator):
    n = navigator
    command(n)
    handle = accept(n)
    assert n.cancel('Cancel at arrival')
    handle.finish(GoalStatus.STATUS_SUCCEEDED)
    assert n.phase == 'CANCELED'
    assert len(n.client.requests) == 1
    assert not n.busy


def test_unconfirmed_cancel_keeps_route_locked(navigator):
    n = navigator
    command(n)
    handle = accept(n)
    n.cancel('Stop')
    handle.cancel_future.set_result(SimpleNamespace(goals_canceling=[]))
    assert n.busy
    assert n.events[-1][0] == 'CANCEL_NOT_CONFIRMED'
    command(n, 'Đi đến B')
    assert len(n.client.requests) == 1
    assert n.events[-1][0] == 'BUSY'


def test_result_transport_error_does_not_release_owned_goal(navigator):
    n = navigator
    command(n)
    handle = accept(n)
    handle.result_future.set_exception(RuntimeError('connection dropped'))
    assert n.busy
    assert n.cancel_requested
    assert handle.cancel_calls == 1
    command(n, 'Đi đến B')
    assert len(n.client.requests) == 1
    assert n.events[-1][0] == 'BUSY'


def test_loss_of_readiness_between_waypoints_stops_next_dispatch(navigator):
    n = navigator
    command(n)
    handle = accept(n)
    n.ready = False
    handle.finish()
    assert len(n.client.requests) == 1
    assert n.phase == 'FAILED'
    assert not n.busy


def test_sending_timeout_retains_ownership_and_cancels_late_goal(navigator):
    n = navigator
    n.check_ready = lambda now: True
    n.get_parameter = lambda key: SimpleNamespace(value=1.0)
    command(n)
    n.sent_at = 0.0
    n.tick()
    assert n.phase == 'CANCELING'
    assert n.busy
    handle = accept(n)
    assert handle.cancel_calls == 1
    handle.finish(GoalStatus.STATUS_CANCELED)
    assert not n.busy
    assert len(n.client.requests) == 1


def test_constructor_initializes_ros_entities_with_real_generated_map():
    """Catch collisions with reserved Node properties and QoS/type errors."""
    from pathlib import Path
    import rclpy
    regions = Path(__file__).resolve().parents[1] / 'config/generated/regions.yaml'
    rclpy.init(args=['--ros-args', '-r', '__node:=voice_navigator_constructor_check',
                    '-p', f'regions_file:={regions}'], domain_id=127)
    n = None
    try:
        n = VoiceNavigator()
        assert n.phase == 'IDLE'
        assert len(n.regions) >= 3
        assert not n.busy
    finally:
        if n is not None:
            n.destroy_node()
        rclpy.shutdown()


def test_send_transport_exception_keeps_uncertain_request_locked(navigator):
    n = navigator
    def fail_send(*args, **kwargs):
        raise RuntimeError('transport failed after send')
    n.client.send_goal_async = fail_send
    command(n)
    assert n.phase == 'ERROR'
    assert n.busy
    assert n.cancel_requested
    assert n.events[-1][0] == 'SEND_UNKNOWN'


def test_accept_response_exception_keeps_uncertain_request_locked(navigator):
    n = navigator
    command(n)
    n.client.responses[0].set_exception(RuntimeError('lost acceptance response'))
    assert n.phase == 'ERROR'
    assert n.busy
    assert n.cancel_requested
    command(n, 'Đi đến B')
    assert n.events[-1][0] == 'BUSY'
    assert len(n.client.requests) == 1


def test_result_request_exception_requests_cancel_and_keeps_lock(navigator):
    n = navigator
    command(n)
    handle = FakeHandle()
    def fail_result():
        raise RuntimeError('result transport failed')
    handle.get_result_async = fail_result
    n.client.responses[0].set_result(handle)
    assert n.busy
    assert n.cancel_requested
    assert handle.cancel_calls == 1


def test_cancel_transport_exception_does_not_release_goal(navigator):
    n = navigator
    command(n)
    handle = accept(n)
    def fail_cancel():
        raise RuntimeError('cancel transport failed')
    handle.cancel_goal_async = fail_cancel
    n.cancel('Stop')
    assert n.busy
    assert n.cancel_requested
    assert n.events[-1][0] == 'CANCEL_NOT_CONFIRMED'
