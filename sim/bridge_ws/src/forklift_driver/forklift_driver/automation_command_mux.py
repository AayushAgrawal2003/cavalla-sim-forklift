#!/usr/bin/env python3
"""Single publisher for ``/automation/command``.

Several stacks (Nav2 twist bridge, line follower, profiled fork height) each
publish a ``ForkliftDirectCommand`` previously all on the same topic; the last
writer won per field and ``brake_interlock_released`` flickered.

This node subscribes to one topic per producer (defaults under
``/automation/command_sources/…``) and republishes a single stream on
``/automation/command`` for ``safety_node``.

Merge rules
-----------
- ``brake_interlock_released`` is the **logical OR** across all *fresh* source
  messages. Any producer that requests release wins.
- ``estop_active`` is likewise OR'd across fresh sources.
- Drive / steer / fork fields come from **at most one** source: the first
  source in ``source_topics`` (highest priority) whose fresh message has
  "motion" above ``motion_epsilon`` on any axis. If none qualify, the output
  is a zero command with only the OR interlock set.
"""

from __future__ import annotations

from typing import Any

import rclpy
from rclpy.node import Node

from forklift_msgs.msg import ForkliftDirectCommand


def _motion_norm(msg: ForkliftDirectCommand) -> float:
    return max(
        abs(msg.drive_speed),
        abs(msg.steering_angle),
        abs(msg.lift_speed),
        abs(msg.tilt_speed),
        abs(msg.side_shift_speed),
        abs(msg.fork_spread_speed),
    )


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


class AutomationCommandMux(Node):
    def __init__(self) -> None:
        super().__init__("automation_command_mux")

        self.declare_parameter("output_topic", "/automation/command")
        self.declare_parameter(
            "source_topics",
            [
                "/automation/command_sources/nav2",
                "/automation/command_sources/line_follower",
                "/automation/command_sources/profiled_fork_height",
                "/automation/command_sources/fork_height",
            ],
        )
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("source_freshness_sec", 0.6)
        self.declare_parameter("motion_epsilon", 0.01)

        out_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        topics = (
            self.get_parameter("source_topics").get_parameter_value().string_array_value
        )
        rate = max(
            1.0,
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value,
        )
        self._fresh = max(
            0.05,
            self.get_parameter("source_freshness_sec").get_parameter_value().double_value,
        )
        self._eps = max(
            0.0,
            self.get_parameter("motion_epsilon").get_parameter_value().double_value,
        )

        self._sources: list[dict[str, Any]] = []
        for t in topics:
            ent: dict[str, Any] = {
                "topic": t,
                "msg": None,
                "stamp": -1.0,
            }
            self._sources.append(ent)
            self.create_subscription(
                ForkliftDirectCommand,
                t,
                self._make_source_callback(ent),
                10,
            )

        self._pub = self.create_publisher(ForkliftDirectCommand, out_topic, 10)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f"automation_command_mux: OR interlock + priority motion merge "
            f"-> {out_topic}; sources={[s['topic'] for s in self._sources]}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _make_source_callback(self, ent: dict[str, Any]):
        def _cb(m: ForkliftDirectCommand) -> None:
            ent["msg"] = m
            ent["stamp"] = self._now()

        return _cb

    def _on_timer(self) -> None:
        now = self._now()
        fresh_msgs: list[ForkliftDirectCommand] = []
        for ent in self._sources:
            msg = ent["msg"]
            st = ent["stamp"]
            if msg is None or st < 0.0 or (now - st) > self._fresh:
                continue
            fresh_msgs.append(msg)

        interlock = bool(
            any(m.brake_interlock_released for m in fresh_msgs)
        )

        out = ForkliftDirectCommand()
        chosen = False
        for ent in self._sources:
            msg = ent["msg"]
            if msg is None:
                continue
            st = ent["stamp"]
            if st < 0.0 or (now - st) > self._fresh:
                continue
            if _motion_norm(msg) > self._eps:
                out = _clone_cmd(msg)
                chosen = True
                break

        if not chosen:
            out = ForkliftDirectCommand()
        out.brake_interlock_released = interlock
        if fresh_msgs:
            out.estop_active = any(m.estop_active for m in fresh_msgs)
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AutomationCommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
