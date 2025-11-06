#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from compliant_controllers_msgs.msg import CartesianCommand
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped


class TestCartesianCommand(Node):
    def __init__(self):
        super().__init__('test_cartesian_command_sender')
        # Parameters
        self.declare_parameter('base_frame', 'fr3_link0')
        self.declare_parameter('ee_frame', 'fr3_link8')
        self.declare_parameter('dx', 0.0)
        self.declare_parameter('dy', 0.0)
        self.declare_parameter('dz', 0.05)
        self.declare_parameter('k_lin', 300.0)
        self.declare_parameter('k_rot', 20.0)

        self.cmd_pub = self.create_publisher(CartesianCommand, 'cartesian_command', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.timer_cb)
        self.sent = False
        self.get_logger().info('Waiting for transform to end-effector...')

    def build_impedance(self, k_lin=300.0, k_rot=20.0):
        k_pos = [float(k_lin),0.0,0.0, 0.0,float(k_lin),0.0, 0.0,0.0,float(k_lin)]
        k_ori = [float(k_rot),0.0,0.0, 0.0,float(k_rot),0.0, 0.0,0.0,float(k_rot)]
        d_lin = 2.0*math.sqrt(k_lin)
        d_rot = 2.0*math.sqrt(k_rot)
        d_pos = [d_lin,0.0,0.0, 0.0,d_lin,0.0, 0.0,0.0,d_lin]
        d_ori = [d_rot,0.0,0.0, 0.0,d_rot,0.0, 0.0,0.0,d_rot]
        return (k_pos, k_ori, d_pos, d_ori)

    def lookup_transform(self):
        base = self.get_parameter('base_frame').get_parameter_value().string_value
        ee = self.get_parameter('ee_frame').get_parameter_value().string_value
        try:
            return self.tf_buffer.lookup_transform(base, ee, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().debug(f'Transform {base}->{ee} not available yet: {e}')
            return None

    def timer_cb(self):
        if self.sent:
            return
        tf = self.lookup_transform()
        if tf is None:
            return
        base = self.get_parameter('base_frame').get_parameter_value().string_value
        ee = self.get_parameter('ee_frame').get_parameter_value().string_value
        dx = self.get_parameter('dx').get_parameter_value().double_value
        dy = self.get_parameter('dy').get_parameter_value().double_value
        dz = self.get_parameter('dz').get_parameter_value().double_value
        k_lin = self.get_parameter('k_lin').get_parameter_value().double_value
        k_rot = self.get_parameter('k_rot').get_parameter_value().double_value

        cmd = CartesianCommand()
        # Current pose target with offsets
        cmd.pose.position.x = tf.transform.translation.x + dx
        cmd.pose.position.y = tf.transform.translation.y + dy
        cmd.pose.position.z = tf.transform.translation.z + dz
        cmd.pose.orientation = tf.transform.rotation  # keep orientation
        cmd.velocity.linear.x = 0.0
        cmd.velocity.linear.y = 0.0
        cmd.velocity.linear.z = 0.0
        cmd.velocity.angular.x = 0.0
        cmd.velocity.angular.y = 0.0
        cmd.velocity.angular.z = 0.0
        cmd.wrench_ff.force.x = 0.0
        cmd.wrench_ff.force.y = 0.0
        cmd.wrench_ff.force.z = 0.0
        cmd.wrench_ff.torque.x = 0.0
        cmd.wrench_ff.torque.y = 0.0
        cmd.wrench_ff.torque.z = 0.0
        k_pos, k_ori, d_pos, d_ori = self.build_impedance(k_lin, k_rot)
        cmd.stiffness_pos = k_pos
        cmd.stiffness_ori = k_ori
        cmd.damping_pos = d_pos
        cmd.damping_ori = d_ori
        # Nullspace fields optional; leave unset
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Published CartesianCommand: target z offset {dz} from current {base}->{ee}.')
        self.sent = True


def main(argv=None):
    rclpy.init(args=argv)
    node = TestCartesianCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
