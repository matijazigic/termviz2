#!/usr/bin/env python3
"""Publish a PointCloud2 to /robot_0/pointcloud2.

Usage:
  python3 pub_pointcloud2.py          # z-colored via turbo gradient (use_rgb: false)
  python3 pub_pointcloud2.py --rgb    # per-point RGB (set use_rgb: true in config)

Publishes at 1 Hz until Ctrl+C.
"""
import argparse
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def build_msg(use_rgb: bool) -> PointCloud2:
    points = []
    for ix in range(-5, 6):
        for iy in range(-5, 6):
            x = ix * 0.3
            y = iy * 0.3
            z = (ix * ix + iy * iy) * 0.01  # bowl shape
            if use_rgb:
                r = min(1.0, abs(x) / 1.5)
                g = min(1.0, abs(y) / 1.5)
                b = 1.0 - r
                points.append((x, y, z, r, g, b))
            else:
                points.append((x, y, z))

    msg = PointCloud2()
    msg.header.frame_id = 'map'
    msg.height = 1
    msg.width = len(points)
    msg.is_bigendian = False
    msg.is_dense = True

    if use_rgb:
        msg.fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        packed = []
        for x, y, z, r, g, b in points:
            ri, gi, bi = int(r * 255), int(g * 255), int(b * 255)
            rgb_int = (ri << 16) | (gi << 8) | bi
            rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
            packed.append(struct.pack('ffff', x, y, z, rgb_float))
        msg.data = b''.join(packed)
    else:
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.data = b''.join(struct.pack('fff', x, y, z) for x, y, z in points)

    msg.row_step = msg.point_step * msg.width
    return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rgb', action='store_true', help='Include RGB field')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('pub_pointcloud2')
    pub = node.create_publisher(PointCloud2, '/robot_0/pointcloud2', 10)

    msg = build_msg(args.rgb)

    def publish():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    timer = node.create_timer(1.0, publish)

    print(f"Publishing PointCloud2 ({msg.width} points, rgb={args.rgb}) — Ctrl+C to stop")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
