"""
Activate the (unmodified) Cavalier line_follower_controller against the sim.

Replaces what the real orchestrator + UI would publish:

  /orchestrator/active_task   AutomationCommand   action='line_follow', RUNNING
  /drop_off/mode              std_msgs/String     'follow_line'

Once these flow, the line_follower_controller (running unmodified in another
terminal) starts emitting commands on /automation/command_sources/line_follower.
The automation_command_mux + safety_node pipe them to /safety/command, which
the MuJoCo bridge consumes — closed loop.

Logs lateral offset and forklift pose every second so you can see the
controller closing in on y=0.

Usage:
    python3 sim/line_follow_runner.py [--duration 30] [--mode follow_line]
"""

from __future__ import annotations

import argparse
import math
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from forklift_msgs.msg import AutomationCommand, ForkliftDirectCommand
from drop_off_msgs.msg import LineState


class LineFollowRunner(Node):
    def __init__(self, mode: str):
        super().__init__('line_follow_runner')
        self._mode = mode
        self._cmd_id = str(uuid.uuid4())

        qos1 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_task = self.create_publisher(
            AutomationCommand, '/orchestrator/active_task', qos1)
        self.pub_mode = self.create_publisher(String, '/drop_off/mode', qos1)

        self._latest_odom: Odometry | None = None
        self._latest_line: LineState | None = None
        self._latest_safety: ForkliftDirectCommand | None = None
        self.create_subscription(Odometry, '/odom',
                                 lambda m: setattr(self, '_latest_odom', m), 10)
        self.create_subscription(LineState, '/drop_off/line_state',
                                 lambda m: setattr(self, '_latest_line', m), 10)
        self.create_subscription(ForkliftDirectCommand, '/safety/command',
                                 lambda m: setattr(self, '_latest_safety', m), 1)

        # Re-publish active_task + mode at 5 Hz so the controller never
        # sees a stale (>3 s) heartbeat and reverts to idle.
        self.create_timer(0.2, self._heartbeat)

        self.get_logger().info(
            f"line_follow_runner: command_id={self._cmd_id} mode={mode}")

    def _heartbeat(self) -> None:
        task = AutomationCommand()
        task.command_id = self._cmd_id
        task.source = 'sim_runner'
        task.action = 'line_follow'
        task.status = AutomationCommand.STATUS_RUNNING
        task.detail = ''
        task.cmd = ForkliftDirectCommand()
        self.pub_task.publish(task)
        self.pub_mode.publish(String(data=self._mode))

    def report(self) -> str:
        odom = self._latest_odom
        ls = self._latest_line
        sc = self._latest_safety
        if odom is None or ls is None:
            return 'waiting...'
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        # yaw from quaternion (z,w only since pitch/roll ~ 0)
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w
        yaw = math.degrees(math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz))
        sc_drv = sc.drive_speed if sc else 0.0
        sc_str = sc.steering_angle if sc else 0.0
        sc_brk = sc.brake_interlock_released if sc else False
        return (f'pos=({x:+6.2f},{y:+6.2f}) yaw={yaw:+5.1f}° '
                f'line_off={ls.guide_offset_m:+.3f}m '
                f'cmd[drv={sc_drv:+.3f} steer={sc_str:+.3f} brake_rel={sc_brk}]')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--mode', default='follow_line',
                   choices=['idle', 'yaw_align', 'x_correct', 'follow_line', 'back_out'])
    p.add_argument('--duration', type=float, default=30.0,
                   help='seconds to keep the task active before stopping')
    args = p.parse_args()

    rclpy.init()
    node = LineFollowRunner(args.mode)

    t_end = time.monotonic() + args.duration
    last_log = 0.0
    try:
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_log > 1.0:
                node.get_logger().info(node.report())
                last_log = now
    except KeyboardInterrupt:
        pass

    # Tell the controller we're done so it stops driving.
    node._mode = 'idle'
    for _ in range(5):
        node._heartbeat()
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
