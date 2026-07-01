import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64, Bool
from rcl_interfaces.msg import SetParametersResult


ARMED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

CONTROL_SUB_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class WrenchMerger(Node):

    def __init__(self):
        super().__init__('wrench_merger')

        # Parameters
        self.declare_parameter('manual_wrench_topic', '/rov/wrench_manual')
        self.declare_parameter('depth_heave_topic', '/ctrl/depth_heave')
        self.declare_parameter('depth_active_topic', '/ctrl/depth_active')
        self.declare_parameter('position_force_topic', '/ctrl/position_force')
        self.declare_parameter('attitude_torque_topic', '/ctrl/attitude_torque')
        self.declare_parameter('output_wrench_topic', '/rov/wrench_cmd')
        self.declare_parameter('armed_topic', '/rov/armed')
        self.declare_parameter('publish_rate', 50.0)

        # manual heave override threshold
        self.declare_parameter('manual_heave_override_threshold', 0.05)
        self.declare_parameter('manual_xy_override_threshold', 0.05)
        self.declare_parameter('manual_yaw_override_threshold', 0.02)
        self.declare_parameter('manual_wrench_timeout_sec', 0.5)

        self.manual_wrench_topic = self.get_parameter(
            'manual_wrench_topic').get_parameter_value().string_value
        self.depth_heave_topic = self.get_parameter(
            'depth_heave_topic').get_parameter_value().string_value
        self.depth_active_topic = self.get_parameter(
            'depth_active_topic').get_parameter_value().string_value
        self.position_force_topic = self.get_parameter(
            'position_force_topic').get_parameter_value().string_value
        self.attitude_torque_topic = self.get_parameter(
            'attitude_torque_topic').get_parameter_value().string_value
        self.output_wrench_topic = self.get_parameter(
            'output_wrench_topic').get_parameter_value().string_value
        self.armed_topic = self.get_parameter(
            'armed_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value
        self.manual_heave_override_threshold = self.get_parameter(
            'manual_heave_override_threshold').get_parameter_value().double_value
        self.manual_xy_override_threshold = self.get_parameter(
            'manual_xy_override_threshold').get_parameter_value().double_value
        self.manual_yaw_override_threshold = self.get_parameter(
            'manual_yaw_override_threshold').get_parameter_value().double_value
        self.manual_wrench_timeout_sec = self.get_parameter(
            'manual_wrench_timeout_sec').get_parameter_value().double_value

        # State
        self.last_manual_wrench = Wrench()
        self.last_manual_wrench_time = None
        self.last_depth_heave = 0.0
        self.depth_active = False
        self.last_position_force = Wrench()
        self.last_attitude_torque = Wrench()

        self.manual_received = False
        self.depth_received = False
        self.depth_active_received = False
        self.position_received = False
        self.attitude_received = False
        self.armed_received = False
        self.armed = False
        self._zero_log_counter = 0
        self._merge_log_counter = 0

        # Subscribers
        self.manual_sub = self.create_subscription(
            Wrench,
            self.manual_wrench_topic,
            self.manual_wrench_callback,
            CONTROL_SUB_QOS
        )

        self.depth_sub = self.create_subscription(
            Float64,
            self.depth_heave_topic,
            self.depth_heave_callback,
            CONTROL_SUB_QOS
        )
        self.depth_active_sub = self.create_subscription(
            Bool,
            self.depth_active_topic,
            self.depth_active_callback,
            CONTROL_SUB_QOS
        )

        self.position_sub = self.create_subscription(
            Wrench,
            self.position_force_topic,
            self.position_force_callback,
            CONTROL_SUB_QOS
        )

        self.attitude_sub = self.create_subscription(
            Wrench,
            self.attitude_torque_topic,
            self.attitude_torque_callback,
            CONTROL_SUB_QOS
        )

        self.armed_sub = self.create_subscription(
            Bool,
            self.armed_topic,
            self.armed_callback,
            ARMED_QOS
        )

        # Publisher
        self.wrench_pub = self.create_publisher(
            Wrench,
            self.output_wrench_topic,
            10
        )

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.05
        self.timer = self.create_timer(period, self.publish_merged_wrench)

        self.add_on_set_parameters_callback(self.on_parameter_update)
        self.get_logger().info('WrenchMerger initialized')
        self.get_logger().info(f'  manual_wrench_topic             = {self.manual_wrench_topic}')
        self.get_logger().info(f'  depth_heave_topic               = {self.depth_heave_topic}')
        self.get_logger().info(f'  depth_active_topic              = {self.depth_active_topic}')
        self.get_logger().info(f'  position_force_topic            = {self.position_force_topic}')
        self.get_logger().info(f'  attitude_torque_topic           = {self.attitude_torque_topic}')
        self.get_logger().info(f'  output_wrench_topic             = {self.output_wrench_topic}')
        self.get_logger().info(f'  armed_topic                     = {self.armed_topic}')
        self.get_logger().info(f'  publish_rate                    = {self.publish_rate:.1f} Hz')
        self.get_logger().info(f'  manual_heave_override_threshold = {self.manual_heave_override_threshold:.3f}')
        self.get_logger().info(f'  manual_xy_override_threshold    = {self.manual_xy_override_threshold:.3f}')
        self.get_logger().info(f'  manual_yaw_override_threshold   = {self.manual_yaw_override_threshold:.3f}')
        self.get_logger().info(f'  manual_wrench_timeout_sec       = {self.manual_wrench_timeout_sec:.3f}')
        self.get_logger().info('  initial armed state             = False')

    def manual_wrench_callback(self, msg: Wrench):
        self.last_manual_wrench = msg
        self.last_manual_wrench_time = self.get_clock().now()
        self.manual_received = True

    def depth_heave_callback(self, msg: Float64):
        self.last_depth_heave = msg.data
        self.depth_received = True

    def depth_active_callback(self, msg: Bool):
        self.depth_active = bool(msg.data)
        self.depth_active_received = True

    def position_force_callback(self, msg: Wrench):
        self.last_position_force = msg
        self.position_received = True

    def attitude_torque_callback(self, msg: Wrench):
        self.last_attitude_torque = msg
        self.attitude_received = True

    def armed_callback(self, msg: Bool):
        prev = self.armed
        self.armed = bool(msg.data)
        self.armed_received = True

        if prev != self.armed:
            self.get_logger().info(f'armed state changed: {self.armed}')

    def publish_zero_wrench(self):
        self.wrench_pub.publish(Wrench())

    def manual_wrench_is_fresh(self) -> bool:
        if not self.manual_received:
            return False
        if self.manual_wrench_timeout_sec <= 0.0:
            return True
        if self.last_manual_wrench_time is None:
            return False

        age = (
            self.get_clock().now() - self.last_manual_wrench_time
        ).nanoseconds * 1e-9
        return age <= self.manual_wrench_timeout_sec

    def publish_merged_wrench(self):
        # armed 신호를 아직 못 받았으면 안전하게 0 유지
        if not self.armed_received:
            self.publish_zero_wrench()
            self._zero_log_counter += 1
            if self._zero_log_counter % 100 == 1:
                self.get_logger().warn('armed state not received yet -> publishing zero wrench')
            self.get_logger().debug('armed state not received yet -> publishing zero wrench')
            return

        # disarm이면 어떤 자동제어가 살아있어도 최종 출력은 무조건 0
        if not self.armed:
            self.publish_zero_wrench()
            self._zero_log_counter += 1
            if self._zero_log_counter % 100 == 1:
                self.get_logger().warn('DISARMED -> publishing zero wrench')
            self.get_logger().debug('DISARMED -> publishing zero wrench')
            return

        manual_wrench = self.last_manual_wrench if self.manual_wrench_is_fresh() else Wrench()
        merged = copy.deepcopy(manual_wrench)

        manual_surge = manual_wrench.force.x
        manual_sway = manual_wrench.force.y
        manual_heave = manual_wrench.force.z

        if abs(manual_surge) > self.manual_xy_override_threshold:
            merged.force.x = manual_surge
        else:
            merged.force.x = self.last_position_force.force.x

        if abs(manual_sway) > self.manual_xy_override_threshold:
            merged.force.y = manual_sway
        else:
            merged.force.y = self.last_position_force.force.y

        # BlueROV-style depth hold:
        # when depth control is active, joystick heave is consumed by the
        # depth controller as a climb/descent-rate request. The final heave
        # output stays automatic so releasing the stick immediately holds depth.
        if self.depth_active:
            merged.force.z = self.last_depth_heave + self.last_position_force.force.z
            heave_mode = 'AUTO_DEPTH_HOLD'
        elif abs(manual_heave) > self.manual_heave_override_threshold:
            merged.force.z = manual_heave
            heave_mode = 'MANUAL_HEAVE_OVERRIDE'
        else:
            merged.force.z = self.last_depth_heave + self.last_position_force.force.z
            heave_mode = 'AUTO_DEPTH_PLUS_POSITION_COMP'

        # Attitude auto control overrides roll/pitch torques
        merged.torque.x = self.last_attitude_torque.torque.x
        merged.torque.y = self.last_attitude_torque.torque.y

        # Keep manual yaw command responsive. If user yaw stick is active, manual wins.
        manual_yaw = manual_wrench.torque.z
        if abs(manual_yaw) > self.manual_yaw_override_threshold:
            merged.torque.z = manual_yaw
        else:
            merged.torque.z = self.last_attitude_torque.torque.z

        self.wrench_pub.publish(merged)
        self._merge_log_counter += 1
        if self._merge_log_counter % 100 == 1:
            self.get_logger().info(
                f'merged wrench: heave_mode={heave_mode}, '
                f'manual_heave={manual_heave:.4f}, '
                f'depth_heave={self.last_depth_heave:.4f}, '
                f'out_force_z={merged.force.z:.4f}'
            )

        # self.get_logger().debug(
        #     f'armed={self.armed}, '
        #     f'heave_mode={heave_mode}, '
        #     f'manual_received={self.manual_received}, '
        #     f'depth_received={self.depth_received}, '
        #     f'attitude_received={self.attitude_received}, '
        #     f'manual_heave={manual_heave:.3f}, auto_heave={self.last_depth_heave:.3f}, '
        #     f'fx={merged.force.x:.3f}, fy={merged.force.y:.3f}, fz={merged.force.z:.3f}, '
        #     f'tx={merged.torque.x:.3f}, ty={merged.torque.y:.3f}, tz={merged.torque.z:.3f}'
        # )

    def on_parameter_update(self, params):
        try:
            recreate_timer = False

            for p in params:
                if p.name == 'publish_rate':
                    self.publish_rate = float(p.value)
                    recreate_timer = True
                elif p.name == 'manual_heave_override_threshold':
                    self.manual_heave_override_threshold = float(p.value)
                elif p.name == 'manual_xy_override_threshold':
                    self.manual_xy_override_threshold = float(p.value)
                elif p.name == 'manual_yaw_override_threshold':
                    self.manual_yaw_override_threshold = float(p.value)
                elif p.name == 'manual_wrench_timeout_sec':
                    self.manual_wrench_timeout_sec = float(p.value)

            if recreate_timer:
                self.timer.cancel()
                period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.05
                self.timer = self.create_timer(period, self.publish_merged_wrench)

            self.get_logger().info('WrenchMerger parameters updated at runtime')
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))


def main(args=None):
    rclpy.init(args=args)
    node = WrenchMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
