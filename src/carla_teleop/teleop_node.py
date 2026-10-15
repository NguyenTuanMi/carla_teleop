#!/usr/bin/env python3
"""
CARLA keyboard teleoperation node.

Reads keyboard input from the terminal (no window, no pygame) and publishes
CarlaEgoVehicleControl messages. View your sensors in Foxglove.

Key bindings
------------
  W / UP      : throttle
  S / DOWN    : brake
  A / LEFT    : steer left
  D / RIGHT   : steer right
  Q           : toggle reverse
  Space       : hand brake (hold)
  P           : toggle autopilot
  Ctrl+C      : quit
"""

import sys
import tty
import termios
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Bool


THROTTLE_STEP = 0.05
BRAKE_STEP    = 0.10
STEER_STEP    = 0.03
STEER_MAX     = 0.7

KEY_MAP = {
    'w': 'throttle',  '\x1b[A': 'throttle',
    's': 'brake',     '\x1b[B': 'brake',
    'a': 'left',      '\x1b[D': 'left',
    'd': 'right',     '\x1b[C': 'right',
    'q': 'reverse',
    ' ': 'handbrake',
    'p': 'autopilot',
}

HELP = """
-- CARLA Teleop --
  W / UP    : throttle
  S / DOWN  : brake
  A / LEFT  : steer left
  D / RIGHT : steer right
  Q         : toggle reverse
  Space     : hand brake
  P         : toggle autopilot
  Ctrl+C    : quit
"""


def _read_key():
    """Read one keypress from stdin (blocking). Returns a string."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class CarlaTeleopNode(Node):

    def __init__(self):
        super().__init__('carla_teleop')

        self.declare_parameter('role_name', 'hero')
        self.declare_parameter('publish_rate', 20.0)

        role = self.get_parameter('role_name').value
        rate = self.get_parameter('publish_rate').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._ctrl_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            f'/carla/{role}/vehicle_control_cmd',
            qos,
        )
        self._autopilot_pub = self.create_publisher(
            Bool,
            f'/carla/{role}/enable_autopilot',
            qos,
        )

        self._throttle    = 0.0
        self._brake       = 0.0
        self._steer_cache = 0.0
        self._reverse     = False
        self._hand_brake  = False
        self._autopilot   = False
        self._gear        = 1
        self._lock        = threading.Lock()

        self._timer = self.create_timer(1.0 / rate, self._publish)

        self._running = True
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.get_logger().info(
            f'CARLA Teleop ready -> /carla/{role}/vehicle_control_cmd'
        )
        print(HELP)

    def _keyboard_loop(self):
        while self._running:
            try:
                key = _read_key()
            except Exception:
                break

            action = KEY_MAP.get(key.lower())
            if action is None:
                continue

            with self._lock:
                if action == 'throttle':
                    self._throttle = min(self._throttle + THROTTLE_STEP, 1.0)
                    self._brake    = 0.0

                elif action == 'brake':
                    self._brake    = min(self._brake + BRAKE_STEP, 1.0)
                    self._throttle = 0.0

                elif action == 'left':
                    self._steer_cache = (
                        0.0 if self._steer_cache > 0
                        else self._steer_cache - STEER_STEP
                    )
                    self._steer_cache = max(-STEER_MAX, self._steer_cache)

                elif action == 'right':
                    self._steer_cache = (
                        0.0 if self._steer_cache < 0
                        else self._steer_cache + STEER_STEP
                    )
                    self._steer_cache = min(STEER_MAX, self._steer_cache)

                elif action == 'reverse':
                    self._reverse = not self._reverse
                    self._gear    = -1 if self._reverse else 1
                    self.get_logger().info(
                        f"Reverse: {'ON' if self._reverse else 'OFF'}"
                    )

                elif action == 'handbrake':
                    self._hand_brake = not self._hand_brake
                    self.get_logger().info(
                        f"Hand brake: {'ON' if self._hand_brake else 'OFF'}"
                    )

                elif action == 'autopilot':
                    self._autopilot = not self._autopilot
                    msg = Bool()
                    msg.data = self._autopilot
                    self._autopilot_pub.publish(msg)
                    self.get_logger().info(
                        f"Autopilot: {'ON' if self._autopilot else 'OFF'}"
                    )

    def _publish(self):
        with self._lock:
            self._throttle    = max(0.0, self._throttle - THROTTLE_STEP)
            self._brake       = max(0.0, self._brake    - BRAKE_STEP)
            self._steer_cache *= 0.7

            msg = CarlaEgoVehicleControl()
            msg.throttle          = float(self._throttle)
            msg.steer             = float(round(self._steer_cache, 3))
            msg.brake             = float(self._brake)
            msg.hand_brake        = self._hand_brake
            msg.reverse            = self._reverse
            msg.gear               = self._gear
            msg.manual_gear_shift  = False

        self._ctrl_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarlaTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
