#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from robot_module_msgs.msg import CartesianCommand
from tf2_ros import Buffer, TransformListener

class DirectionalImpedanceDemo(Node):
    """Publish a single CartesianCommand with anisotropic stiffness.

    Goal: Stiff in world Z, in XY plane have a compliant axis at angle (default 45°)
    and a stiffer orthogonal axis. Orientation stiffness uniform.
    """
    def __init__(self):
        super().__init__('directional_impedance_demo')
        # Parameters
        self.declare_parameter('base_frame', 'fr3_link0')
        self.declare_parameter('ee_frame', 'fr3_link8')
        # Allow either integer or float overrides from CLI via dynamic_typing
        self.declare_parameter('angle_deg', 45.0, ParameterDescriptor(dynamic_typing=True))   # compliant axis direction in world XY
        self.declare_parameter('k_compliant', 200.0)
        self.declare_parameter('k_stiff_xy', 1500.0)
        self.declare_parameter('k_stiff_z', 2500.0)
        self.declare_parameter('k_rot', 20.0)
        self.declare_parameter('d_ratio', 1.0)  # scaling vs critical damping
        self.declare_parameter('dz', 0.0)  # optional small offset

        self.pub = self.create_publisher(CartesianCommand, 'cartesian_command', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sent = False
        self.timer = self.create_timer(0.5, self._tick)
        self.get_logger().info('Waiting for transform to end-effector...')

    # Utility builders -------------------------------------------------
    @staticmethod
    def _build_impedance_matrix_linear(k_par, k_orth, k_z, angle_rad):
        """Return flattened 3x3 linear stiffness rotated about Z.

        In plane: compliant axis (k_par) at angle angle_rad from world X.
        Orthogonal axis (k_orth) at angle+90deg.
        Z axis stiffness k_z (world aligned).
        """
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        k11 = c*c*k_par + s*s*k_orth
        k12 = c*s*(k_par - k_orth)
        k22 = s*s*k_par + c*c*k_orth
        return [k11, k12, 0.0,
                k12, k22, 0.0,
                0.0, 0.0, k_z]

    @staticmethod
    def _critical_damping(k):
        return 2.0 * math.sqrt(max(k, 0.0))

    def _build_impedance(self, k_par, k_orth, k_z, k_rot, angle_rad, d_ratio):
        k_lin = self._build_impedance_matrix_linear(k_par, k_orth, k_z, angle_rad)
        # Orientation block isotropic
        k_ori = [k_rot,0.0,0.0, 0.0,k_rot,0.0, 0.0,0.0,k_rot]
        # Damping: scale critical damping
        d_par = d_ratio * self._critical_damping(k_par)
        d_orth = d_ratio * self._critical_damping(k_orth)
        d_z = d_ratio * self._critical_damping(k_z)
        d_lin = self._build_impedance_matrix_linear(d_par, d_orth, d_z, angle_rad)
        d_rot = d_ratio * self._critical_damping(k_rot)
        d_ori = [d_rot,0.0,0.0, 0.0,d_rot,0.0, 0.0,0.0,d_rot]
        return (k_lin, k_ori, d_lin, d_ori)

    # Main loop --------------------------------------------------------
    def _lookup_transform(self):
        base = self.get_parameter('base_frame').get_parameter_value().string_value
        ee = self.get_parameter('ee_frame').get_parameter_value().string_value
        try:
            return self.tf_buffer.lookup_transform(base, ee, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as ex:
            self.get_logger().debug(f'TF {base}->{ee} not yet available: {ex}')
            return None

    def _tick(self):
        if self.sent:
            return
        tf = self._lookup_transform()
        if tf is None:
            return
        # Retrieve angle parameter tolerating int/float
        angle_deg = float(self.get_parameter('angle_deg').value)
        k_par = self.get_parameter('k_compliant').get_parameter_value().double_value
        k_orth = self.get_parameter('k_stiff_xy').get_parameter_value().double_value
        k_z = self.get_parameter('k_stiff_z').get_parameter_value().double_value
        k_rot = self.get_parameter('k_rot').get_parameter_value().double_value
        d_ratio = self.get_parameter('d_ratio').get_parameter_value().double_value
        dz = self.get_parameter('dz').get_parameter_value().double_value

        angle_rad = math.radians(angle_deg)
        cmd = CartesianCommand()
        # Position target: keep XY, apply optional dz offset
        cmd.pose_des.position.x = tf.transform.translation.x
        cmd.pose_des.position.y = tf.transform.translation.y
        cmd.pose_des.position.z = tf.transform.translation.z + dz
        cmd.pose_des.orientation = tf.transform.rotation
        # Zero velocity / wrench
        cmd.velocity_des.linear.x = 0.0
        cmd.velocity_des.linear.y = 0.0
        cmd.velocity_des.linear.z = 0.0
        cmd.velocity_des.angular.x = 0.0
        cmd.velocity_des.angular.y = 0.0
        cmd.velocity_des.angular.z = 0.0
        cmd.wrench_ff.force.x = 0.0
        cmd.wrench_ff.force.y = 0.0
        cmd.wrench_ff.force.z = 0.0
        cmd.wrench_ff.torque.x = 0.0
        cmd.wrench_ff.torque.y = 0.0
        cmd.wrench_ff.torque.z = 0.0
        k_lin, k_ori, d_lin, d_ori = self._build_impedance(k_par, k_orth, k_z, k_rot, angle_rad, d_ratio)
        cmd.stiffness_pos = k_lin
        cmd.stiffness_ori = k_ori
        cmd.damping_pos = d_lin
        cmd.damping_ori = d_ori
        self.pub.publish(cmd)
        self.get_logger().info(
            f'Published anisotropic CartesianCommand: compliant axis {angle_deg:.1f}deg, k_par={k_par}, k_orth={k_orth}, k_z={k_z}, k_rot={k_rot}.')
        self.sent = True


def main(argv=None):
    rclpy.init(args=argv)
    node = DirectionalImpedanceDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Usage example:
# ros2 run compliant_controllers test_cartesian_directional_impedance.py --ros-args -p angle_deg:=45 -p k_compliant:=200 -p k_stiff_xy:=0 -p k_stiff_z:=2500 -p k_rot:=20
