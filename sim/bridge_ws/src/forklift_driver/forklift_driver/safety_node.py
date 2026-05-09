import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float32, Bool
from forklift_msgs.msg import ForkliftDirectCommand

# /safety/command_source (std_msgs/UInt8 .data) — mux selection for UIs / logging.
COMMAND_SOURCE_STOP_IDLE = 0
COMMAND_SOURCE_TELEOP = 1
COMMAND_SOURCE_AUTO = 2
COMMAND_SOURCE_STOP_TELEOP_TRIP = 3

class ForkliftSafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # --- Parameters & State ---
        self.heartbeat_timeout_sec = 0.75  # 750ms
        self.command_timeout_sec = 1.0    # How old a teleop command can be before it's considered stale

        self.last_heartbeat_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        # --- MUX STATE ---
        self.latest_teleop_cmd = ForkliftDirectCommand()
        self.latest_auto_cmd = ForkliftDirectCommand()
        self.last_auto_cmd_time = self.get_clock().now()
        self.AUTO_CMD_TIMEOUT_SEC = 0.5  # Freshness window for /automation/command

        self.latest_auto_effort = 0.0
        self.last_auto_effort_time = self.get_clock().now()  # Freshness of /automation/auto_lift_effort
        self.AUTO_EFFORT_TIMEOUT_SEC = 0.5

        self.auto_fork_canceled = False
        self.last_teleop_activity_time = self.get_clock().now()

        self.TELEOP_PRIORITY_TIMEOUT = 3.0  # Pause auto for 3s after teleop
        self.ACTIVITY_THRESHOLD = 0.01      # Threshold to consider teleop "active"

        self.current_status_code = 3  # Default to 3 (Disconnected/Unsafe)
        self.was_safe = False         # Edge-detection for cleaner logging
        self.last_source = None       # 'teleop' | 'auto' | None, for log edges

        # --- Brake Interlock Lockout (teleop-only) ---
        # Stale teleop heartbeat must not leave teleop's cached interlock in a
        # "released" state. This lockout only clamps the teleop branch; the
        # automation branch carries its own brake_interlock_released.
        self.BRAKE_LOCKOUT_TIMEOUT_SEC = 2.0
        self.brake_interlock_locked = False
        self.last_interlock_state = False

        # --- Orchestrator teleop lockout ---
        # When True the orchestrator has locked teleop out so automation
        # commands get priority over teleop at the mux level.
        self.orchestrator_teleop_locked = False

        # --- Subscribers ---
        # Teleop channel
        self.create_subscription(ForkliftDirectCommand, '/teleop/command', self.teleop_cb, 1)
        # Adamo Web Heartbeat (0=Safe, 1=Unfocused, 2=High Latency, 3=Disconnected)
        self.create_subscription(UInt8, '/teleop/safety_status', self.heartbeat_cb, 1)

        # Automation channels
        # Full-control command from the automation stack (drive, steer, forks, brake interlock).
        self.create_subscription(ForkliftDirectCommand, '/automation/command', self.auto_cmd_cb, 1)
        # Dedicated PID lift effort from fork_height_controller (merged onto lift when in auto mode).
        self.create_subscription(Float32, '/automation/auto_lift_effort', self.auto_effort_cb, 10)
        self.create_subscription(Float32, '/automation/target_fork_height', self.auto_target_cb, 1)

        # Orchestrator teleop lockout signal
        self.create_subscription(Bool, '/orchestrator/teleop_locked', self.teleop_lock_cb, 1)

        # --- Publishers ---
        self.safe_pub = self.create_publisher(ForkliftDirectCommand, '/safety/command', 1)
        self.teleop_state_pub = self.create_publisher(Bool, '/safety/teleop_state', 1)
        self.command_source_pub = self.create_publisher(UInt8, '/safety/command_source', 1)

        # --- Watchdog Timer (50Hz to match PID + encoder rates) ---
        self.create_timer(0.02, self.watchdog_loop)

        self.get_logger().info("Safety Multiplexer Initialized. Awaiting Heartbeat...")

    # ------------------------------------------------------------------ #
    #  Callbacks
    # ------------------------------------------------------------------ #

    def heartbeat_cb(self, msg: UInt8):
        self.current_status_code = msg.data
        self.last_heartbeat_time = self.get_clock().now()

    def auto_cmd_cb(self, msg: ForkliftDirectCommand):
        self.latest_auto_cmd = msg
        self.last_auto_cmd_time = self.get_clock().now()

    def auto_effort_cb(self, msg: Float32):
        # fork_height_controller publishes NaN when PID is inactive; treat as "no effort".
        v = msg.data
        if isinstance(v, float) and math.isnan(v):
            self.latest_auto_effort = 0.0
        else:
            self.latest_auto_effort = v
        self.last_auto_effort_time = self.get_clock().now()

    def auto_target_cb(self, msg: Float32):
        # If we receive a new target, we assume the operator wants auto fork to resume
        if self.auto_fork_canceled:
            self.get_logger().info("New Auto Target Received: Resuming Auto Fork control.")
            self.auto_fork_canceled = False

    def teleop_lock_cb(self, msg: Bool):
        prev = self.orchestrator_teleop_locked
        self.orchestrator_teleop_locked = msg.data
        if msg.data and not prev:
            self.get_logger().info("Orchestrator LOCKED teleop — automation gets command priority.")
        elif not msg.data and prev:
            self.get_logger().info("Orchestrator UNLOCKED teleop — normal priority restored.")

    def teleop_cb(self, msg: ForkliftDirectCommand):
        self.latest_teleop_cmd = msg
        self.last_cmd_time = self.get_clock().now()

        is_driving = (abs(msg.drive_speed) > self.ACTIVITY_THRESHOLD or
                      abs(msg.steering_angle) > self.ACTIVITY_THRESHOLD)

        is_forking = (abs(msg.lift_speed) > self.ACTIVITY_THRESHOLD or
                      abs(msg.tilt_speed) > self.ACTIVITY_THRESHOLD or
                      abs(msg.side_shift_speed) > self.ACTIVITY_THRESHOLD or
                      abs(msg.fork_spread_speed) > self.ACTIVITY_THRESHOLD)

        if is_driving or is_forking:
            self.last_teleop_activity_time = self.get_clock().now()

            # If the operator specifically moved the forks, cancel auto PID lift
            # until the next auto target arrives. The full auto command channel
            # is NOT gated by this — it has its own authority.
            if is_forking and not self.auto_fork_canceled:
                self.get_logger().warn("Teleop Fork Intervention: Canceling Auto Fork PID until new target.")
                self.auto_fork_canceled = True

    # ------------------------------------------------------------------ #
    #  Safety predicates
    # ------------------------------------------------------------------ #

    def check_teleop_safety(self) -> tuple[bool, str]:
        """Is the teleop connection healthy? Returns (is_safe, reason)."""
        now = self.get_clock().now()
        time_since_heartbeat = (now - self.last_heartbeat_time).nanoseconds / 1e9
        time_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9

        # 0 = safe, 1 = unfocused (unsafe), 2 = high latency (safe for now), 3 = disconnected (unsafe)
        if self.current_status_code in [1, 3]:
            return False, f"Status Code {self.current_status_code} (Unfocused/Disconnected)"
        if time_since_heartbeat > self.heartbeat_timeout_sec:
            return False, f"Heartbeat Stale ({time_since_heartbeat:.2f}s)"
        if time_since_cmd > self.command_timeout_sec:
            return False, f"Command Stale ({time_since_cmd:.2f}s)"
        return True, ""

    def auto_command_fresh(self) -> bool:
        age = (self.get_clock().now() - self.last_auto_cmd_time).nanoseconds / 1e9
        return age <= self.AUTO_CMD_TIMEOUT_SEC

    def auto_effort_fresh(self) -> bool:
        age = (self.get_clock().now() - self.last_auto_effort_time).nanoseconds / 1e9
        return age <= self.AUTO_EFFORT_TIMEOUT_SEC

    # ------------------------------------------------------------------ #
    #  Watchdog / Mux
    # ------------------------------------------------------------------ #

    def watchdog_loop(self):
        now = self.get_clock().now()
        time_since_activity = (now - self.last_teleop_activity_time).nanoseconds / 1e9
        time_since_heartbeat = (now - self.last_heartbeat_time).nanoseconds / 1e9

        # --- Teleop Brake Interlock Lockout ---
        # Only affects the teleop passthrough path; auto owns its own interlock.
        if time_since_heartbeat > self.BRAKE_LOCKOUT_TIMEOUT_SEC:
            if not self.brake_interlock_locked:
                self.get_logger().warn("Heartbeat absent >2s: Teleop brake interlock LOCKED.")
            self.brake_interlock_locked = True

        current_interlock = self.latest_teleop_cmd.brake_interlock_released
        if self.brake_interlock_locked:
            if current_interlock and not self.last_interlock_state:
                # Rising edge after a falling edge: operator deliberately re-armed
                self.brake_interlock_locked = False
                self.get_logger().info("Teleop brake interlock re-armed by operator.")
        self.last_interlock_state = current_interlock

        # --- Source Selection ---
        teleop_safe, teleop_reason = self.check_teleop_safety()
        teleop_active = (time_since_activity < self.TELEOP_PRIORITY_TIMEOUT)
        auto_cmd_ok = self.auto_command_fresh()

        source = None
        fault_reason = ""

        if self.orchestrator_teleop_locked and auto_cmd_ok:
            source = 'auto'
        elif teleop_active and teleop_safe and not self.orchestrator_teleop_locked:
            source = 'teleop'
        elif auto_cmd_ok:
            source = 'auto'
        elif teleop_active and teleop_safe and self.orchestrator_teleop_locked:
            source = 'teleop'
        elif teleop_active and not teleop_safe:
            fault_reason = f"TELEOP SAFETY TRIP: {teleop_reason}"
        else:
            fault_reason = "NO FRESH COMMAND SOURCE"

        if source == 'teleop':
            command_source_byte = COMMAND_SOURCE_TELEOP
        elif source == 'auto':
            command_source_byte = COMMAND_SOURCE_AUTO
        elif teleop_active and not teleop_safe:
            command_source_byte = COMMAND_SOURCE_STOP_TELEOP_TRIP
        else:
            command_source_byte = COMMAND_SOURCE_STOP_IDLE

        self.command_source_pub.publish(UInt8(data=command_source_byte))

        # --- Build Command ---
        if source == 'teleop':
            mux_cmd = self.latest_teleop_cmd
            if self.brake_interlock_locked:
                # Clamp teleop's cached interlock if heartbeat has been gone.
                mux_cmd = self._clone_cmd(mux_cmd)
                mux_cmd.brake_interlock_released = False
            self._publish_safe(mux_cmd, source)

        elif source == 'auto':
            mux_cmd = self._clone_cmd(self.latest_auto_cmd)
            effort_fresh = self.auto_effort_fresh()
            merge_pid = (
                effort_fresh
                and not self.auto_fork_canceled
                and abs(mux_cmd.lift_speed) < 1e-6
            )
            if merge_pid:
                mux_cmd.lift_speed = float(self.latest_auto_effort)
            auto_cmd_age = (
                self.get_clock().now() - self.last_auto_cmd_time
            ).nanoseconds / 1e9
            effort_age = (
                self.get_clock().now() - self.last_auto_effort_time
            ).nanoseconds / 1e9
            self.get_logger().info(
                f"safety auto mux: merge_pid_lift={merge_pid} "
                f"auto_fork_canceled={self.auto_fork_canceled} "
                f"effort_fresh={effort_fresh} "
                f"latest_effort={self.latest_auto_effort:+.4f} "
                f"age(cmd={auto_cmd_age:.2f}s effort={effort_age:.2f}s) "
                f"cached_auto_lift={self.latest_auto_cmd.lift_speed:+.4f} "
                f"-> out_lift={mux_cmd.lift_speed:+.4f} "
                f"orch_lock={self.orchestrator_teleop_locked}",
                throttle_duration_sec=0.4,
            )
            self._publish_safe(mux_cmd, source)

        else:
            self.get_logger().warn(
                f"safety STOP path: {fault_reason} "
                f"(teleop_active={teleop_active} teleop_safe={teleop_safe} "
                f"auto_cmd_ok={auto_cmd_ok} orch_lock={self.orchestrator_teleop_locked})",
                throttle_duration_sec=0.5,
            )
            self._publish_stop(fault_reason)

        # --- State Publish ---
        self.teleop_state_pub.publish(
            Bool(data=teleop_safe and not self.brake_interlock_locked)
        )

    # ------------------------------------------------------------------ #
    #  Publish helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _clone_cmd(src: ForkliftDirectCommand) -> ForkliftDirectCommand:
        out = ForkliftDirectCommand()
        out.drive_speed = src.drive_speed
        out.steering_angle = src.steering_angle
        out.lift_speed = src.lift_speed
        out.tilt_speed = src.tilt_speed
        out.side_shift_speed = src.side_shift_speed
        out.fork_spread_speed = src.fork_spread_speed
        out.estop_active = src.estop_active
        out.accel_time_s = src.accel_time_s
        out.decel_time_s = src.decel_time_s
        out.brake_interlock_released = src.brake_interlock_released
        return out

    def _publish_safe(self, cmd: ForkliftDirectCommand, source: str):
        if not self.was_safe or self.last_source != source:
            self.get_logger().info(f"SYSTEM SAFE. Source = {source.upper()}.")
            self.was_safe = True
            self.last_source = source
        self.safe_pub.publish(cmd)

    def _publish_stop(self, reason: str):
        if self.was_safe:
            self.get_logger().warn(f"SAFETY TRIP! Reason: {reason}. Clamping to 0.0.")
            self.was_safe = False
            self.last_source = None
        stop_cmd = ForkliftDirectCommand()
        stop_cmd.brake_interlock_released = False
        self.safe_pub.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
