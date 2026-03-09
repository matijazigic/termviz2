#!/usr/bin/env python3
"""Publish a MarkerArray to /robot_0/visualization_marker_array at 1 Hz."""
import rclpy
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray


def make_marker(ns, id_, type_, x, y, r, g, b, sx=0.3, sy=0.3, sz=0.3, lifetime_sec=0):
    m = Marker()
    m.header.frame_id = 'map'
    m.ns = ns
    m.id = id_
    m.type = type_
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
    m.lifetime = Duration(sec=lifetime_sec, nanosec=0)
    return m


def main():
    rclpy.init()
    node = rclpy.create_node('pub_marker_array')
    pub = node.create_publisher(MarkerArray, '/robot_0/visualization_marker_array', 10)

    arr = MarkerArray()
    arr.markers = [
        make_marker('shapes', 0, Marker.SPHERE,    1.0, 0.0, 1.0, 0.0, 0.0),
        make_marker('shapes', 1, Marker.SPHERE,    2.0, 1.0, 0.0, 1.0, 0.0),
        make_marker('shapes', 2, Marker.CUBE,      0.0, 2.0, 0.0, 0.0, 1.0),
        make_marker('shapes', 3, Marker.ARROW,     1.5, 1.5, 1.0, 1.0, 0.0,
                    sx=0.5, sy=0.1, sz=0.1, lifetime_sec=0),
    ]

    def publish():
        stamp = node.get_clock().now().to_msg()
        for m in arr.markers:
            m.header.stamp = stamp
        pub.publish(arr)

    node.create_timer(1.0, publish)

    print("Publishing MarkerArray — Ctrl+C to stop")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
