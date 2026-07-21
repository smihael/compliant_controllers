#!/usr/bin/env python3
"""Set Cartesian stiffness blocks without changing the pose reference."""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class StiffnessFrameOffsetTest(Node):
    def __init__(self):
        super().__init__('stiffness_frame_offset_test', namespace='/fr3')
        self.declare_parameter('position_stiffness', 500.0)
        self.declare_parameter('orientation_stiffness', 0.0)
        self.declare_parameter('publish_delay', 2.0)

        self.position_publisher = self.create_publisher(
            Float64MultiArray, 'stiffness_pos', 1)
        self.orientation_publisher = self.create_publisher(
            Float64MultiArray, 'stiffness_ori', 1)
        self.published = False
        self.timer = self.create_timer(
            float(self.get_parameter('publish_delay').value), self._publish)
        self.get_logger().info(
            'Will update stiffness only; the controller Cartesian reference is unchanged.')

    def _publish(self):
        if self.published:
            return
        kp = float(self.get_parameter('position_stiffness').value)
        kr = float(self.get_parameter('orientation_stiffness').value)
        if not all(math.isfinite(value) for value in (kp, kr)) or kp < 0.0 or kr < 0.0:
            raise ValueError('stiffness values must be finite and non-negative')

        position_matrix = [kp, 0.0, 0.0, 0.0, kp, 0.0, 0.0, 0.0, kp]
        orientation_matrix = [kr, 0.0, 0.0, 0.0, kr, 0.0, 0.0, 0.0, kr]

        position_message = Float64MultiArray()
        position_message.data = position_matrix
        orientation_message = Float64MultiArray()
        orientation_message.data = orientation_matrix
        self.position_publisher.publish(position_message)
        self.orientation_publisher.publish(orientation_message)
        self.published = True
        self.get_logger().info(
            f'Published stiffness-only update: Kp={kp}, Kr={kr}. '
            'No Cartesian pose reference was sent.')


def main(args=None):
    rclpy.init(args=args)
    node = StiffnessFrameOffsetTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
