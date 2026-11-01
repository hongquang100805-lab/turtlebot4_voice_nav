"""Latched functional-region boundaries and labelled safe destinations in RViz."""
from pathlib import Path

import cv2
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from .map_regions import OccupancyMap, load_regions
from .voice_navigator import default_regions


class SemanticVisualizer(Node):
    def __init__(self):
        super().__init__('semantic_visualizer')
        self.declare_parameter('regions_file', default_regions())
        source = Path(self.get_parameter('regions_file').value).expanduser().resolve()
        self.data = load_regions(source)
        occupancy = OccupancyMap.from_yaml(self.data['map_yaml'])
        mask = Path(self.data['mask_file'])
        if not mask.is_absolute():
            mask = source.parent / mask
        with np.load(mask, allow_pickle=False) as stored:
            labels = stored['labels']
        out = MarkerArray(); clear = Marker(); clear.action = Marker.DELETEALL
        out.markers.append(clear)
        colors = [(.98, .62, .12), (.2, .8, .38), (.28, .4, .96), (.9, .3, .75)]
        number = 0
        for i, region in enumerate(self.data['regions']):
            color = colors[i % len(colors)]
            contours, _ = cv2.findContours((labels == region['numeric_label']).astype(np.uint8),
                                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                marker = self.marker(number, 'boundaries', Marker.LINE_STRIP, color)
                marker.scale.x = .045; number += 1
                points = cv2.approxPolyDP(contour, 1.0, True).reshape(-1, 2)
                if len(points) > 1:
                    for col, row in np.vstack([points, points[0]]):
                        x, y = occupancy.pixel_to_world(int(col), int(row))
                        marker.points.append(Point(x=float(x), y=float(y), z=.04))
                    out.markers.append(marker)
            goal = region['goal']
            marker = self.marker(number, 'goals', Marker.SPHERE, color); number += 1
            marker.pose.position.x = float(goal['x']); marker.pose.position.y = float(goal['y'])
            marker.pose.position.z = .15
            marker.scale.x = marker.scale.y = marker.scale.z = .25
            out.markers.append(marker)
            marker = self.marker(number, 'names', Marker.TEXT_VIEW_FACING, color); number += 1
            marker.pose.position.x = float(goal['x']); marker.pose.position.y = float(goal['y']) + .4
            marker.pose.position.z = .35; marker.scale.z = .7
            # ASCII IDs remain legible with standard RViz fonts; full names are in YAML/status.
            marker.text = region['id']
            out.markers.append(marker)
        self.markers = out
        self.pub = self.create_publisher(MarkerArray, '/semantic_regions/markers', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub.publish(self.markers)
        self.get_logger().info(f"Hiển thị {len(self.data['regions'])} vùng trên frame map")

    @staticmethod
    def marker(number, namespace, kind, color):
        marker = Marker(); marker.header.frame_id = 'map'; marker.id = number
        marker.ns = namespace; marker.type = kind; marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        return marker


def main(args=None):
    rclpy.init(args=args); node = None
    try:
        node = SemanticVisualizer(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
