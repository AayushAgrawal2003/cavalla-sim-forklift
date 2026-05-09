#!/usr/bin/env python3
"""Drop-off line-follower controller.

Always running; emits a steady stream of `ForkliftDirectCommand`
messages on ``output_topic`` (default
``/automation/command_sources/line_follower``) so
``forklift_driver/automation_command_mux`` can merge with other producers
before ``/automation/command``. By default it publishes
ZERO commands -- the truck does not move. It only emits non-zero
commands when:

  1. `/drop_off/mode` is set to one of `yaw_align`, `x_correct`, or
     `follow_line`,
  2. AND a fresh `LineState` is available, AND the relevant detection
     for that mode is present (e.g. `guide_detected` for follow modes),
  3. AND the per-mode safety gates pass (travel limit, line-lost
     timeout, stop decider).

Modes
-----
- `idle` (default, no movement)
- `yaw_align`     -- pivot in place at +/- pi/2 steering until
                     |guide_yaw_rad| < yaw_done_rad
- `x_correct`     -- drive reverse with the same lateral PD as
                     `follow_line`, but abort after `max_travel_m`
                     metres of odometry travel
- `follow_line`   -- drive reverse, lateral PD; stop when the
                     `StopDecider` triggers

All commands are merged by
`forklift_driver/automation_command_mux`, which publishes the single
`/automation/command` stream for `safety_node` -> `driver_node`.

`brake_interlock_released` on inactive ticks follows parameter
``brake_interlock_released_when_inactive`` (default false). The mux takes the
**logical OR** of interlock across fresh sources, so fork-height automation
can still release the brake while this node is idle.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from forklift_msgs.msg import AutomationCommand, ForkliftDirectCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from drop_off_msgs.msg import LineState

from drop_off.stop_decider import LineStopDecider


# Allowed modes. `idle` is the safe default the controller assumes when
# no message has been received yet, when the mode topic goes stale, or
# when an unknown mode string is published.
VALID_MODES = ('idle', 'yaw_align', 'x_correct', 'follow_line', 'back_out')


class LineFollowerController(Node):
    def __init__(self) -> None:
        super().__init__('line_follower_controller')

        # ---- Topics ----
        self.declare_parameter('line_state_topic', '/drop_off/line_state')
        self.declare_parameter('mode_topic', '/drop_off/mode')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_topic', '/automation/command_sources/line_follower')
        # When True, zero / inactive publishes set interlock released (legacy).
        # Default false: mux OR merges interlock from profiled fork height, etc.
        self.declare_parameter('brake_interlock_released_when_inactive', False)

        # ---- Loop rate & timeouts ----
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('line_state_timeout_sec', 0.3)
        self.declare_parameter('line_lost_timeout_sec', 0.5)

        # ---- Speed / steering caps (bench-safe defaults) ----
        # Normalized [-1, 1] on the wire. 0.05 is ~5% throttle which,
        # with the existing 2.5 m/s wheel cap in twist_to_forklift, is
        # very slow. Override per-test if needed.
        self.declare_parameter('low_drive_norm', 0.05)
        self.declare_parameter('max_steer_norm', 0.3)
        # Hard ceilings applied just before publish, regardless of
        # mode-specific intentions, to guarantee a slow bench truck.
        self.declare_parameter('max_drive_norm', 0.1)

        # ---- Lateral / yaw PD gains for follow_line / x_correct ----
        # `delta = k_lat * (offset_px / image_width) + k_yaw * yaw_rad`
        self.declare_parameter('k_lat', 0.6)
        self.declare_parameter('k_yaw', 1.2)

        # ---- Mode-specific parameters ----
        # x_correct: hard cap on travelled distance from start (m).
        self.declare_parameter('max_travel_m', 2.0)
        # x_correct: hard cap on |offset_norm|. If the lateral error
        # ever exceeds this fraction of image width while x_correct is
        # active, the steering sign is almost certainly wrong (line
        # has wandered to the very edge of the frame). Drop to idle
        # and log loudly. Default 0.45 = nearly off-frame.
        self.declare_parameter('x_correct_offset_abort_norm', 0.45)

        # yaw_align tolerances and direction.
        self.declare_parameter('yaw_done_rad', 0.05)
        # +1 = drive forward while pivoting, -1 = reverse. The forklift
        # passive-axle pivot point doesn't translate at delta=+/-pi/2
        # regardless of sign; the sign just chooses rotation direction.
        # Default is reverse (-1) to match the rest of the approach
        # being a reverse manoeuvre.
        self.declare_parameter('yaw_align_drive_sign', -1)
        # Steering magnitude for yaw_align. Defaults to full lock (1.0)
        # so the truck actually pivots about the passive axle instead
        # of arcing across the floor. Kept independent from
        # `max_steer_norm`, which intentionally limits follow_line /
        # x_correct to gentle inputs. Don't lower this below ~0.9 unless
        # you're deliberately testing partial-lock pivoting.
        self.declare_parameter('yaw_align_steer_norm', 1.0)
        # Divergence guard: track |guide_yaw_rad| over this window.
        # If after `yaw_align_divergence_check_sec` the smallest |yaw|
        # observed is no smaller than the |yaw| at mode entry minus
        # `yaw_align_divergence_min_progress_rad`, the steering sign is
        # almost certainly wrong (truck is spinning further off-axis).
        # Drop to idle and log loudly so the operator can flip the
        # sign and try again instead of waiting on the e-stop.
        self.declare_parameter('yaw_align_divergence_check_sec', 2.0)
        self.declare_parameter('yaw_align_divergence_min_progress_rad', 0.02)
        # Hard timeout for the manoeuvre. If we haven't converged in
        # this many seconds, drop to idle. Catches sensor drift, weak
        # gain, or a stuck truck.
        self.declare_parameter('yaw_align_max_duration_sec', 20.0)

        # ---- StopDecider (LineStopDecider params live here for now) ----
        # The band is in fractions of image height from the bottom.
        # Default [0.0, 1.0] = anywhere in the frame counts as a stop
        # sample. Narrow it later if false positives at one end of the
        # frame become an issue (e.g. set high=0.7 to ignore a stop
        # tape that's still very close to the top of the image).
        self.declare_parameter('stop_band_low_frac', 0.0)
        self.declare_parameter('stop_band_high_frac', 1.0)
        # Debounce. Trigger stop only after this many consecutive
        # *unique* line_state frames satisfy the band check. The
        # decider dedupes by header stamp so the count reflects camera
        # frames (~18-20 Hz), not control ticks (50 Hz). At ~20 fps
        # 5 frames ≈ 250 ms.
        self.declare_parameter('stop_min_consec_frames', 5)

        # ---- Direction sign for follow / x_correct ----
        # +1 = drive forward, -1 = drive reverse. Default reverse since
        # the user described approaching the conveyor backwards.
        self.declare_parameter('drive_direction_sign', -1)

        # ---- ArUco distance guard ----
        self.declare_parameter(
            'aruco_guard_topic', '/aruco_distance_guard/below_threshold'
        )
        self.declare_parameter('aruco_guard_modes', ['follow_line'])

        # ---- back_out per-mode tuning ----
        # Front-steered tricycle (drive at rear, steered castor at front,
        # camera at front) is kinematically asymmetric: forward driving
        # is closed-loop STABLE under PD line-following, reverse driving
        # is closed-loop UNSTABLE. The classical reverse-stabilising
        # controller flips ONE of the feedback signs (offset or yaw,
        # not both — mirroring would cancel back to forward) AND uses
        # smaller gains because the open-loop is marginally stable.
        # See e.g. Sastry/Murray "Nonholonomic Motion Planning",
        # tractor-trailer reverse-control papers.
        #
        # Independent per-term signs let you find which one truly needs
        # to flip on this hardware (theory says yaw, practice may say
        # otherwise on a real vehicle with castor offset / weight
        # transfer / steered-wheel friction).
        #
        # delta = back_out_offset_sign * (drive_dir * k_lat * off_n)
        #       + back_out_yaw_sign    *               (k_yaw * yaw_rad)
        # then * back_out_steering_sign (legacy multiplier, kept for
        # convenience to flip everything in one shot).
        self.declare_parameter('back_out_steering_sign', -1.0)
        self.declare_parameter('back_out_offset_sign', 1.0)
        self.declare_parameter('back_out_yaw_sign', 1.0)
        # Reverse driving wants ~0.3-0.5x the forward gains (high gain
        # destabilises). Defaults: half of the bench-tuned follow_line
        # values (k_lat=0.4 -> 0.2, k_yaw=3.5 -> 1.75). Set < 0 to fall
        # back to follow_line's k_lat / k_yaw.
        self.declare_parameter('back_out_k_lat', 0.2)
        self.declare_parameter('back_out_k_yaw', 1.75)

        # ---- State ----
        self._orchestrator_active = False
        self._last_active_stamp: float = 0.0
        self._mode: str = 'idle'
        self._last_line_state: Optional[LineState] = None
        self._last_line_stamp: float = 0.0
        self._last_guide_seen_stamp: float = 0.0

        self._mode_entered_stamp: float = 0.0
        self._mode_start_x: Optional[float] = None
        self._mode_start_y: Optional[float] = None
        self._latest_x: float = 0.0
        self._latest_y: float = 0.0

        # yaw_align divergence tracking. Populated on _enter_mode.
        self._yaw_align_initial_abs: Optional[float] = None
        self._yaw_align_min_abs: float = float('inf')

        self._aruco_below_threshold: bool = False

        self._stop_decider = LineStopDecider()
        # Separate decider for back_out: watches yellow_stop_detected /
        # yellow_stop_distance_px so the back-away phase doesn't pick up
        # the red drop-off stop tape that triggered follow_line earlier.
        self._yellow_stop_decider = LineStopDecider(
            detected_attr='yellow_stop_detected',
            distance_attr='yellow_stop_distance_px',
        )

        # Task tracking — latched from /orchestrator/active_task.
        self._task_command_id: str = ''
        self._task_action: str = ''
        # Pending terminal status to emit on the next tick (e.g. COMPLETED on
        # stop-line detection, ABORTED on an internal failure).
        self._pending_terminal_status: Optional[int] = None
        self._pending_terminal_detail: str = ''
        # Completed-broadcast latch.
        self.declare_parameter('completed_broadcast_sec', 1.0)
        self._completed_command_id: str = ''
        self._completed_action: str = ''
        self._completed_detail: str = ''
        self._completed_until_s: float = 0.0

        # ---- pubs/subs ----
        self.create_subscription(
            AutomationCommand, '/orchestrator/active_task', self._active_cb, 10,
        )
        self.create_subscription(
            LineState,
            self.get_parameter('line_state_topic').value,
            self._on_line_state,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter('mode_topic').value,
            self._on_mode,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._on_odom,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter('aruco_guard_topic').value,
            self._on_aruco_guard,
            10,
        )
        self._pub_cmd = self.create_publisher(
            AutomationCommand,
            self.get_parameter('output_topic').value,
            10,
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(1.0, rate), self._on_timer)

        # Per-tick logging is too noisy at 50 Hz, so we only log on
        # transitions and at most once per second for steady-state.
        self._last_log_stamp: float = 0.0
        self._last_logged_mode: Optional[str] = None
        self._last_logged_active: Optional[bool] = None

        self.get_logger().info(
            'line_follower_controller ready: '
            f'{self.get_parameter("line_state_topic").value} + '
            f'{self.get_parameter("mode_topic").value} -> '
            f'{self.get_parameter("output_topic").value} '
            f'@ {rate:.1f} Hz, default mode=idle'
        )

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def _on_line_state(self, msg: LineState) -> None:
        self._last_line_state = msg
        self._last_line_stamp = self._now()
        if msg.guide_detected:
            self._last_guide_seen_stamp = self._last_line_stamp

    def _on_mode(self, msg: String) -> None:
        new_mode = (msg.data or '').strip().lower()
        if new_mode not in VALID_MODES:
            self.get_logger().warn(
                f'unknown mode "{msg.data}", ignoring (valid: {VALID_MODES})'
            )
            return
        if new_mode != self._mode:
            self.get_logger().info(f'mode {self._mode} -> {new_mode}')
            self._mode = new_mode
            self._enter_mode(new_mode)

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_x = float(msg.pose.pose.position.x)
        self._latest_y = float(msg.pose.pose.position.y)

    def _on_aruco_guard(self, msg: Bool) -> None:
        self._aruco_below_threshold = bool(msg.data)

    def _active_cb(self, msg: AutomationCommand) -> None:
        prev = self._orchestrator_active
        running = (
            msg.action == 'line_follow'
            and msg.status == AutomationCommand.STATUS_RUNNING
        )
        self._orchestrator_active = running
        if running:
            self._last_active_stamp = self._now()
            if msg.command_id and msg.command_id != self._task_command_id:
                self._task_command_id = msg.command_id
                self._task_action = msg.action
            if not prev:
                self.get_logger().info(
                    f"Orchestrator activated line follower (id={self._task_command_id})"
                )
        else:
            if prev and self._mode != 'idle':
                self.get_logger().info("Orchestrator deactivated — reverting to idle")
                self._mode = 'idle'
            self._task_command_id = ''
            self._task_action = ''

    def _complete_mode(self, reason: str) -> None:
        previous = self._mode
        self._mode = 'idle'
        self._orchestrator_active = False
        # Mark COMPLETED so the next tick emits the terminal status under the
        # current command_id. The id stays latched until we publish.
        if self._task_command_id and self._pending_terminal_status is None:
            self._pending_terminal_status = AutomationCommand.STATUS_COMPLETED
            self._pending_terminal_detail = f'{previous}: {reason}'
        self.get_logger().info(f'mode {previous} completed ({reason}); reverting to idle')

    def _abort_mode(self, reason: str) -> None:
        previous = self._mode
        self._mode = 'idle'
        self._orchestrator_active = False
        if self._task_command_id and self._pending_terminal_status is None:
            self._pending_terminal_status = AutomationCommand.STATUS_ABORTED
            self._pending_terminal_detail = f'{previous}: {reason}'
        self.get_logger().info(f'mode {previous} aborted ({reason}); reverting to idle')

    def _enter_mode(self, mode: str) -> None:
        self._mode_entered_stamp = self._now()
        # Latch the start pose so x_correct can enforce its travel cap
        # from the beginning of THIS run rather than mode-history.
        self._mode_start_x = self._latest_x
        self._mode_start_y = self._latest_y
        # Reset yaw_align divergence trackers; they latch on the first
        # in-mode tick that has a fresh guide reading.
        self._yaw_align_initial_abs = None
        self._yaw_align_min_abs = float('inf')
        # Reset the stop decider so a stale consecutive count from a
        # previous run can't immediately trigger a stop on this one.
        self._stop_decider.reset()
        self._yellow_stop_decider.reset()

    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        now = self._now()
        now_mono = time.monotonic()

        # COMPLETED rebroadcast latch.
        if self._completed_command_id and now_mono < self._completed_until_s:
            am = AutomationCommand()
            am.command_id = self._completed_command_id
            am.source = 'line_follower'
            am.action = self._completed_action or 'line_follow'
            am.status = AutomationCommand.STATUS_COMPLETED
            am.detail = self._completed_detail
            self._pub_cmd.publish(am)
            if now_mono >= self._completed_until_s:
                self._completed_command_id = ''
                self._completed_action = ''
                self._completed_detail = ''
            return

        # Pending terminal (from _complete_mode this tick): emit it once and
        # latch for the broadcast window.
        if self._pending_terminal_status is not None and self._task_command_id:
            am = AutomationCommand()
            am.command_id = self._task_command_id
            am.source = 'line_follower'
            am.action = self._task_action or 'line_follow'
            am.status = self._pending_terminal_status
            am.detail = self._pending_terminal_detail
            self._pub_cmd.publish(am)

            self._completed_command_id = self._task_command_id
            self._completed_action = self._task_action or 'line_follow'
            self._completed_detail = self._pending_terminal_detail
            self._completed_until_s = now_mono + float(
                self.get_parameter('completed_broadcast_sec').value
            )
            self._pending_terminal_status = None
            self._pending_terminal_detail = ''
            return

        # Active staleness: if orchestrator stops publishing, revert to idle.
        if self._orchestrator_active:
            active_age = now - self._last_active_stamp
            if active_age > 3.0:
                self.get_logger().warn(
                    f'active topic stale ({active_age:.1f}s > 3.0s); '
                    f'reverting to idle (orchestrator crashed?)'
                )
                self._orchestrator_active = False
                if self._mode != 'idle':
                    self._mode = 'idle'

        cmd, active, reason = self._compute_command(now)

        am = AutomationCommand()
        am.command_id = self._task_command_id
        am.source = 'line_follower'
        am.action = self._task_action if self._task_command_id else ''
        am.status = (
            AutomationCommand.STATUS_RUNNING
            if (active and self._task_command_id)
            else AutomationCommand.STATUS_IDLE
        )
        am.cmd = cmd
        am.detail = reason
        self._pub_cmd.publish(am)
        self._maybe_log(now, active, reason)

    # ------------------------------------------------------------------
    def _compute_command(
        self, now: float
    ) -> tuple[ForkliftDirectCommand, bool, str]:
        cmd = ForkliftDirectCommand()
        if bool(
            self.get_parameter('brake_interlock_released_when_inactive').value
        ):
            cmd.brake_interlock_released = True
        # Defaults: zero drive/steer.

        if self._mode == 'idle':
            return cmd, False, 'idle'

        if not self._orchestrator_active:
            return cmd, False, 'orchestrator inactive'

        guarded_modes = [
            str(m) for m in self.get_parameter('aruco_guard_modes').value
        ]
        if self._aruco_below_threshold and self._mode in guarded_modes:
            self.get_logger().warn(
                f'aruco_distance_guard: marker(s) closer than threshold; '
                f'aborting {self._mode} back to teleop'
            )
            self._abort_mode('aruco distance below threshold')
            return cmd, False, 'aborted: aruco distance below threshold'

        # --- Common gating: need a fresh LineState ---
        line_timeout = float(self.get_parameter('line_state_timeout_sec').value)
        line_age = now - self._last_line_stamp
        if self._last_line_state is None or line_age > line_timeout:
            return cmd, False, f'no fresh line_state (age={line_age:.2f}s)'

        ls = self._last_line_state

        # All modes need at least one fresh line in front of the robot
        # ("line in front" = guide_detected OR a stop line — including
        # the yellow back_out stop). The user requirement: "if there is
        # no line in front of the forklift, I don't want it to move."
        if (
            not ls.guide_detected
            and not ls.stop_detected
            and not ls.yellow_stop_detected
        ):
            return cmd, False, 'no line in view'

        # Decide which stop decider applies to this mode. back_out
        # watches the yellow stop tape; everything else watches the
        # red stop tape that drop_off uses.
        if self._mode == 'back_out':
            decider = self._yellow_stop_decider
            stop_label = 'yellow stop'
        else:
            decider = self._stop_decider
            stop_label = 'stop line'

        should_stop, stop_reason = decider.should_stop(
            ls,
            stop_band_low_frac=float(
                self.get_parameter('stop_band_low_frac').value
            ),
            stop_band_high_frac=float(
                self.get_parameter('stop_band_high_frac').value
            ),
            min_consec_frames=int(
                self.get_parameter('stop_min_consec_frames').value
            ),
        )
        if should_stop:
            self._complete_mode(f'{stop_label}: {stop_reason}')
            return cmd, False, f'{stop_label}: {stop_reason}'

        if self._mode == 'yaw_align':
            return self._yaw_align(cmd, ls)

        if self._mode in ('follow_line', 'x_correct', 'back_out'):
            # Need the guide line specifically.
            if not ls.guide_detected:
                line_lost_timeout = float(
                    self.get_parameter('line_lost_timeout_sec').value
                )
                if (now - self._last_guide_seen_stamp) > line_lost_timeout:
                    return cmd, False, 'guide line lost'
                # Brief flicker; emit zero this tick but don't drop the mode.
                return cmd, False, 'guide flicker'

            if self._mode == 'x_correct':
                travelled = self._distance_from_start()
                cap = float(self.get_parameter('max_travel_m').value)
                if travelled >= cap:
                    self._complete_mode(
                        f'travel cap reached ({travelled:.2f}m >= {cap:.2f}m)'
                    )
                    return (
                        cmd,
                        False,
                        f'travel cap reached ({travelled:.2f}m >= {cap:.2f}m)',
                    )

                # Offset-runaway guard. If |offset_norm| ever exceeds
                # the abort threshold while x_correct is active, the
                # truck is wandering toward the edge of the frame --
                # almost certainly an offset sign error. Drop to idle.
                w = max(1, int(ls.image_width))
                offset_norm_abs = abs(float(ls.guide_offset_px) / float(w))
                abort_norm = float(
                    self.get_parameter('x_correct_offset_abort_norm').value
                )
                if offset_norm_abs > abort_norm:
                    self.get_logger().error(
                        f'x_correct: |offset_norm|={offset_norm_abs:.3f} '
                        f'exceeded abort threshold {abort_norm:.3f}. '
                        f'Truck is wandering off the line -- offset sign is '
                        f'almost certainly wrong. Flip '
                        f'offset_sign_<camera> in line_detector.yaml.'
                    )
                    self._complete_mode('offset runaway')
                    return cmd, False, 'x_correct offset runaway'

            return self._follow_line(cmd, ls)

        # Unknown active mode: be safe.
        return cmd, False, f'unknown active mode {self._mode}'

    # ------------------------------------------------------------------
    def _yaw_align(
        self,
        cmd: ForkliftDirectCommand,
        ls: LineState,
    ) -> tuple[ForkliftDirectCommand, bool, str]:
        yaw = float(ls.guide_yaw_rad)
        abs_yaw = abs(yaw)
        done_tol = float(self.get_parameter('yaw_done_rad').value)
        if abs_yaw < done_tol:
            self._complete_mode(
                f'yaw aligned (|{yaw:.3f}|<{done_tol:.3f})'
            )
            return cmd, False, f'yaw aligned (|{yaw:.3f}|<{done_tol:.3f})'

        # ---- Track progress for divergence / duration guards. ----
        if self._yaw_align_initial_abs is None:
            self._yaw_align_initial_abs = abs_yaw
            self._yaw_align_min_abs = abs_yaw
        elif abs_yaw < self._yaw_align_min_abs:
            self._yaw_align_min_abs = abs_yaw

        elapsed = self._now() - self._mode_entered_stamp

        # ---- Hard duration cap. ----
        max_duration = float(
            self.get_parameter('yaw_align_max_duration_sec').value
        )
        if elapsed > max_duration:
            self.get_logger().error(
                f'yaw_align timeout after {elapsed:.1f}s '
                f'(|yaw|={abs_yaw:.3f}, started at '
                f'{self._yaw_align_initial_abs:.3f})'
            )
            self._complete_mode(f'yaw_align timeout ({elapsed:.1f}s)')
            return cmd, False, f'yaw_align timeout ({elapsed:.1f}s)'

        # ---- Divergence guard. ----
        check_after = float(
            self.get_parameter('yaw_align_divergence_check_sec').value
        )
        min_progress = float(
            self.get_parameter('yaw_align_divergence_min_progress_rad').value
        )
        if (
            elapsed > check_after
            and self._yaw_align_initial_abs is not None
            and (self._yaw_align_initial_abs - self._yaw_align_min_abs)
            < min_progress
        ):
            self.get_logger().error(
                f'yaw_align DIVERGING after {elapsed:.1f}s: '
                f'|yaw| started at {self._yaw_align_initial_abs:.3f} rad, '
                f'best so far {self._yaw_align_min_abs:.3f} rad '
                f'(< {min_progress:.3f} rad progress). Sign is likely '
                f'wrong -- flip yaw_sign_<camera> in line_detector.yaml '
                f'or yaw_align_drive_sign in line_follower.yaml.'
            )
            self._complete_mode('yaw_align diverging (sign wrong?)')
            return cmd, False, 'yaw_align diverging (sign wrong?)'

        # Pivot direction: positive `yaw` means the line tilts clockwise
        # in the image (in controller-frame, after the per-camera
        # yaw_sign flip). Drive in reverse with full steering lock in
        # the matching direction to pivot in place about the passive
        # axle. The exact sign relationship is camera/install-specific;
        # the divergence guard above kicks in if it's wrong.
        steer_mag = float(self.get_parameter('yaw_align_steer_norm').value)
        steer_mag = max(0.0, min(1.0, steer_mag))
        steer_norm = math.copysign(steer_mag, yaw)

        drive_sign = float(self.get_parameter('yaw_align_drive_sign').value)
        drive_sign = 1.0 if drive_sign >= 0.0 else -1.0
        drive_norm = drive_sign * float(self.get_parameter('low_drive_norm').value)

        cmd.drive_speed = self._cap_drive(drive_norm)
        cmd.steering_angle = float(steer_norm)
        cmd.brake_interlock_released = True
        return (
            cmd,
            True,
            f'yaw_align yaw={yaw:+.3f} steer={steer_norm:+.2f} '
            f'drv={cmd.drive_speed:+.3f} '
            f'best=|{self._yaw_align_min_abs:.3f}| t={elapsed:.1f}s',
        )

    # ------------------------------------------------------------------
    def _follow_line(
        self,
        cmd: ForkliftDirectCommand,
        ls: LineState,
    ) -> tuple[ForkliftDirectCommand, bool, str]:
        k_lat = float(self.get_parameter('k_lat').value)
        k_yaw = float(self.get_parameter('k_yaw').value)
        max_steer = float(self.get_parameter('max_steer_norm').value)

        # back_out can override gains independently (see __init__ docstring).
        if self._mode == 'back_out':
            bo_k_lat = float(self.get_parameter('back_out_k_lat').value)
            bo_k_yaw = float(self.get_parameter('back_out_k_yaw').value)
            if bo_k_lat >= 0.0:
                k_lat = bo_k_lat
            if bo_k_yaw >= 0.0:
                k_yaw = bo_k_yaw

        w = max(1, int(ls.image_width))
        offset_norm = float(ls.guide_offset_px) / float(w)
        yaw_rad = float(ls.guide_yaw_rad)

        drive_dir = float(self.get_parameter('drive_direction_sign').value)
        drive_dir = 1.0 if drive_dir >= 0.0 else -1.0
        # back_out drives the OPPOSITE way along the same red guide:
        # follow_line/x_correct approach the conveyor; back_out departs.
        if self._mode == 'back_out':
            drive_dir = -drive_dir

        # Sign reasoning, kept terse here, full derivation in the
        # module docstring of `line_detector.yaml`:
        #
        # * OFFSET term flips with drive direction. When reversing, a
        #   positive lateral offset (line right of desired column)
        #   means the rear is to the left of where it should be. To
        #   shift the rear right we steer LEFT (negative). Forward
        #   driving is the opposite. So delta_offset = drive_dir *
        #   k_lat * offset_norm.
        #
        # * YAW term DOES NOT flip with drive direction in the inherited
        #   reverse-tested follow_line formula. For back_out (the
        #   open-loop unstable reverse-driving case for a front-steered
        #   tricycle), we apply per-term signs and an overall scalar so
        #   you can independently invert the offset feedback, the yaw
        #   feedback, or the whole delta. See __init__ docstring.
        offset_term = drive_dir * k_lat * offset_norm
        yaw_term = k_yaw * yaw_rad

        if self._mode == 'back_out':
            offset_term *= float(self.get_parameter('back_out_offset_sign').value)
            yaw_term *= float(self.get_parameter('back_out_yaw_sign').value)

        delta_norm = offset_term + yaw_term

        if self._mode == 'back_out':
            delta_norm *= float(self.get_parameter('back_out_steering_sign').value)

        delta_norm = max(-max_steer, min(max_steer, delta_norm))

        drive_norm = drive_dir * float(
            self.get_parameter('low_drive_norm').value
        )

        cmd.drive_speed = self._cap_drive(drive_norm)
        cmd.steering_angle = float(delta_norm)
        cmd.brake_interlock_released = True
        return (
            cmd,
            True,
            f'follow off_n={offset_norm:+.3f} yaw={yaw_rad:+.3f} '
            f'-> drv={cmd.drive_speed:+.3f} steer={cmd.steering_angle:+.3f}',
        )

    # ------------------------------------------------------------------
    def _cap_drive(self, drive_norm: float) -> float:
        cap = float(self.get_parameter('max_drive_norm').value)
        cap = max(0.0, min(1.0, cap))
        return float(max(-cap, min(cap, drive_norm)))

    def _distance_from_start(self) -> float:
        if self._mode_start_x is None or self._mode_start_y is None:
            return 0.0
        dx = self._latest_x - self._mode_start_x
        dy = self._latest_y - self._mode_start_y
        return math.hypot(dx, dy)

    # ------------------------------------------------------------------
    def _maybe_log(self, now: float, active: bool, reason: str) -> None:
        # Always log on edge transitions; throttle steady-state.
        if (
            active != self._last_logged_active
            or self._mode != self._last_logged_mode
            or (now - self._last_log_stamp) > 1.0
        ):
            self.get_logger().info(
                f'[mode={self._mode}] '
                f'{"ACTIVE" if active else "ZERO  "} {reason}'
            )
            self._last_log_stamp = now
            self._last_logged_active = active
            self._last_logged_mode = self._mode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineFollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
