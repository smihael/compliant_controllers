#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from compliant_controllers_msgs.msg import CartesianCommand
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.srv import GetParameters


class TestCartesianCommand(Node):
    def __init__(self):
        super().__init__('test_cartesian_command_sender')
        # Parameters
        # Namespace and naming flexibility
        self.declare_parameter('robot_name', 'lbr')
        self.declare_parameter('robot_ns', '')  # Alternative to CLI namespace; if set, will prefix service/topic
        self.declare_parameter('tf_prefix', '')  # Prefix to TF frames (e.g., "lbr")
        self.declare_parameter('tf_prefix_delim', '/')  # Delimiter between prefix and frame name ("/" or "_")

        # Controller/topic parameters
        self.declare_parameter('controller_name', 'cartesian_impedance_controller')
        self.declare_parameter('base_frame', '')  # Default resolved to <robot_name>_link_0
        self.declare_parameter('ee_frame', '')  # Empty means query from controller or default to <robot_name>_link_ee
        self.declare_parameter('dx', 0.0)
        self.declare_parameter('dy', 0.0)
        self.declare_parameter('dz', 0.05)
        self.declare_parameter('k_lin', 300.0)
        self.declare_parameter('k_rot', 20.0)

        # Resolve namespace/topic base: if robot_ns param is set, use it; otherwise rely on node namespace
        # Publisher uses a relative topic so it respects the node's namespace or the remapped namespace.
        self.cmd_pub = self.create_publisher(CartesianCommand, 'cartesian_command', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Resolve default frames based on robot_name if not provided
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        ee_frame_cfg = self.get_parameter('ee_frame').get_parameter_value().string_value
        if not base_frame:
            base_frame = f"{robot_name}_link_0"
            self.set_parameters([rclpy.parameter.Parameter('base_frame', rclpy.Parameter.Type.STRING, base_frame)])
        if not ee_frame_cfg:
            # Will attempt to query from controller; if that fails, default to <robot_name>_link_ee
            ee_frame_cfg = ''
        
        # Try to get ee_frame from controller if not explicitly set
        if not ee_frame_cfg:
            self.get_logger().info('ee_frame not provided, querying from controller...')
            self.query_controller_ee_frame()
            # If still empty after query, set to default <robot_name>_link_ee
            ee_frame_param = self.get_parameter('ee_frame').get_parameter_value().string_value
            if not ee_frame_param:
                self.set_parameters([rclpy.parameter.Parameter('ee_frame', rclpy.Parameter.Type.STRING, f"{robot_name}_link_ee")])
        
        self.timer = self.create_timer(0.5, self.timer_cb)
        self.sent = False
        self.get_logger().info('Waiting for transform to end-effector...')

    def query_controller_ee_frame(self):
        """Query the ee_frame parameter from the controller node."""
        controller_name = self.get_parameter('controller_name').get_parameter_value().string_value
        # Build service name relative to namespace to support namespaced deployments
        # Using relative names ensures the node's namespace (or remapped namespace) is respected
        service_name = f"{controller_name}/get_parameters"
        client = self.create_client(GetParameters, service_name)
        
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Could not reach {controller_name} parameter service at "{service_name}", using default ee_frame')
            robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
            self.set_parameters([rclpy.parameter.Parameter('ee_frame', rclpy.Parameter.Type.STRING, f"{robot_name}_link_ee")])
            return
        
        request = GetParameters.Request()
        request.names = ['ee_frame']
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None and len(future.result().values) > 0:
            ee_frame = future.result().values[0].string_value
            self.set_parameters([rclpy.parameter.Parameter('ee_frame', rclpy.Parameter.Type.STRING, ee_frame)])
            self.get_logger().info(f'Using ee_frame from controller: {ee_frame}')
        else:
            self.get_logger().warn('Could not get ee_frame from controller, using default based on robot_name')
            robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
            self.set_parameters([rclpy.parameter.Parameter('ee_frame', rclpy.Parameter.Type.STRING, f"{robot_name}_link_ee")])

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
        # Apply TF prefix if provided
        tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value
        tf_delim = self.get_parameter('tf_prefix_delim').get_parameter_value().string_value
        if tf_prefix:
            base = f"{tf_prefix}{tf_delim}{base}"
            ee = f"{tf_prefix}{tf_delim}{ee}"
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
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = ee
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
        self.get_logger().info(f'Published CartesianCommand: offset [dx={dx}, dy={dy}, dz={dz}] from current {base}->{ee}.')
        self.sent = True


def main(argv=None):
    rclpy.init(args=argv)
    node = TestCartesianCommand()
    try:
        while rclpy.ok() and not node.sent:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
