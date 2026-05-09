import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from forklift_msgs.msg import ForkliftDirectCommand, ForkliftDriveFeedback
from .can_interface.mbv15_iface import MBV15Interface

DEFAULT_ACCEL_S = 1.0
DEFAULT_DECEL_S = 0.2


class ForkliftDriverNode(Node):
    def __init__(self):
        super().__init__('forklift_driver')

        self.curtis = MBV15Interface(channel='can0', bitrate=250000)
        if self.curtis.connected:
            self.get_logger().info("Connected to MBV15/Curtis Controller on can0")
        else:
            self.get_logger().warn("CAN0 not found. Running in MOCK mode.")

        self.create_subscription(
            ForkliftDirectCommand, '/safety/command', self.command_callback, 1
        )

        self.steering_pub = self.create_publisher(Float32, '/forklift/steering_angle', 1)
        self.drive_feedback_pub = self.create_publisher(
            ForkliftDriveFeedback, '/forklift/drive_feedback', 1
        )
        self.create_timer(0.05, self._feedback_timer)

    def _feedback_timer(self):
        try:
            state = self.curtis.read_feedback()

            steer_msg = Float32()
            steer_msg.data = float(state['steering_rad'])
            self.steering_pub.publish(steer_msg)

            fb = ForkliftDriveFeedback()
            fb.motor_speed_rpm       = int(state['motor_rpm'])
            fb.motor_current_amps    = float(state['current_amps'])
            fb.drive_fault_code      = int(state['drive_fault'])
            fb.battery_percent       = int(state['battery_percent'])
            fb.odometer_cm           = int(state['odometer_cm'])
            fb.estop_active          = bool(state['estop_active'])
            fb.interlock_on          = bool(state['interlock_on'])
            fb.forward_switch        = bool(state['forward_switch'])
            fb.reverse_switch        = bool(state['reverse_switch'])
            fb.mode_auto             = bool(state['mode_auto'])
            fb.electromagnetic_brake = bool(state['electromagnetic_brake'])
            self.drive_feedback_pub.publish(fb)
        except Exception as e:
            self.get_logger().error(f"CAN feedback read failed: {e}", throttle_duration_sec=5.0)

    def command_callback(self, msg: ForkliftDirectCommand):
        accel = msg.accel_time_s if msg.accel_time_s > 0.0 else DEFAULT_ACCEL_S
        decel = msg.decel_time_s if msg.decel_time_s > 0.0 else DEFAULT_DECEL_S

        self.get_logger().info(f"lift: {msg.lift_speed}")

        try:
            self.curtis.send_commands(
                msg.drive_speed,
                msg.steering_angle,
                msg.lift_speed,
                msg.tilt_speed,
                msg.side_shift_speed,
                accel_s=accel,
                decel_s=decel,
                interlock_released=msg.brake_interlock_released,
            )
        except Exception as e:
            self.get_logger().error(f"CAN send failed: {e}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down driver. Commanding STOP.")
        try:
            node.curtis.stop_all()
        except Exception as e:
            node.get_logger().error(f"CAN stop_all failed during shutdown: {e}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
