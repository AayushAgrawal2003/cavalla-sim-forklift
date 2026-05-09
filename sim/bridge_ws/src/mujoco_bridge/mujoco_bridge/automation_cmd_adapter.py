"""Tiny adapter — bridges the line_follower's `AutomationCommand` output to
the `ForkliftDirectCommand` topic shape that `automation_command_mux`
subscribes to.

The line_follower (drop_off pkg) publishes AutomationCommand on
``/automation/command_sources/line_follower``; the mux subscribes as
ForkliftDirectCommand. That mismatch exists in upstream Cavalier
(transition between dev branches). This adapter unwraps the inner
``cmd`` field and republishes on a sibling topic the mux can be
configured to listen on.

  in:  /automation/command_sources/line_follower         (AutomationCommand)
  out: /automation/command_sources/line_follower_cmd     (ForkliftDirectCommand)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from forklift_msgs.msg import AutomationCommand, ForkliftDirectCommand


class AutomationCommandAdapter(Node):
    def __init__(self):
        super().__init__('automation_cmd_adapter')
        self.declare_parameter('input_topic', '/automation/command_sources/line_follower')
        self.declare_parameter('output_topic', '/automation/command_sources/line_follower_cmd')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self._pub = self.create_publisher(ForkliftDirectCommand, out_topic, 10)
        self.create_subscription(AutomationCommand, in_topic, self._on_msg, 10)

        self.get_logger().info(f'adapter: {in_topic} (AutomationCommand) -> {out_topic} (ForkliftDirectCommand)')

    def _on_msg(self, msg: AutomationCommand) -> None:
        self._pub.publish(msg.cmd)


def main():
    rclpy.init()
    node = AutomationCommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
