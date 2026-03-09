#!/usr/bin/env python3
"""Publish a single Marker to /robot_0/visualization_marker at 1 Hz."""
import rclpy
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker


def main():
    rclpy.init()
    node = rclpy.create_node('pub_marker')
    pub = node.create_publisher(Marker, '/robot_0/visualization_marker', 10)

    m = Marker()
    m.header.frame_id = 'map'
    m.ns = 'test'
    m.id = 0
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.position.x = 1.0
    m.pose.position.y = 1.0
    m.pose.orientation.w = 1.0
    m.scale.x = 0.5
    m.scale.y = 0.1
    m.scale.z = 0.1
    m.color.r = 1.0
    m.color.g = 0.5
    m.color.b = 0.0
    m.color.a = 1.0
    m.lifetime = Duration(sec=0, nanosec=0)

    def publish():
        m.header.stamp = node.get_clock().now().to_msg()
        pub.publish(m)

    node.create_timer(1.0, publish)

    print("Publishing Marker — Ctrl+C to stop")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
