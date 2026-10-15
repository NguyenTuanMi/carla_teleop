#!/usr/bin/env python3
"""
CARLA keyboard teleoperation node.

Reads keyboard input from the terminal and publishes
CarlaEgoVehicleControl messages. View your sensors in Foxglove.

Key bindings
------------
    W           : throttle
    S           : brake
    A           : steer left
    D           : steer right
  Q           : toggle reverse
  Space       : hand brake (hold)
  P           : toggle autopilot
  Ctrl+C      : quit
"""

import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Bool


KEY_MAP = {
    '\x1b[A': 'throttle',
    '\x1bOA': 'throttle',
    '\x1b[B': 'brake',
    '\x1bOB': 'brake',
    '\x1b[D': 'left',
    '\x1bOD': 'left',
    '\x1b[C': 'right',
    '\x1bOC': 'right',
    'q': 'reverse',
    ' ': 'handbrake',
    'p': 'autopilot',
}

HELP = """
-- CARLA Teleop --
    UP        : throttle
    DOWN      : brake
    LEFT      : steer left
    RIGHT     : steer right
  Q         : toggle reverse
  Space     : hand brake
  P         : toggle autopilot
  Ctrl+C    : quit
"""


class CarlaTeleopNode(Node):

    def __init__(self):
        super().__init__('carla_teleop')

        self.declare_parameter('role_name', 'hero')
        self.declare_parameter('publish_rate', 60.0)
        self.declare_parameter('throttle_rate', 1.2)
        self.declare_parameter('brake_rate', 4.0)
        self.declare_parameter('steer_rate', 1.0)
        self.declare_parameter('steer_max', 0.7)
        self.declare_parameter('coast_brake', 0.25)
        legacy_timeout = self.declare_parameter('key_timeout', 0.60).value
        self.declare_parameter('longitudinal_key_timeout', legacy_timeout)
        self.declare_parameter('steering_key_timeout', 0.80)
        self.declare_parameter('toggle_debounce', 0.25)

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

        self._timer = self.create_timer(1.0 / max(float(rate), 1.0), self._publish)

        self._running = True
        self._key_times = {}
        self._toggle_times = {}
        self._last_publish = time.monotonic()
        self._input_fd = self._open_terminal()
        self._input_old = None
        self._input_buffer = ''
        if self._input_fd is not None:
            self._input_old = termios.tcgetattr(self._input_fd)
            tty.setcbreak(self._input_fd)

        self.get_logger().info(
            f'CARLA Teleop ready -> /carla/{role}/vehicle_control_cmd'
        )
        print(HELP)

    def _handle_key(self, action, now):
        if action in {'throttle', 'brake', 'left', 'right', 'handbrake'}:
            with self._lock:
                self._key_times[action] = now
                if action in {'left', 'right'}:
                    # Terminal input has no key-up event. Refresh the active
                    # longitudinal command while a steering key is repeating,
                    # matching manual_control.py's held-key behavior.
                    key_timeout = float(
                        self.get_parameter('longitudinal_key_timeout').value
                    )
                    for longitudinal in ('throttle', 'brake'):
                        timestamp = self._key_times.get(longitudinal)
                        if timestamp is not None and now - timestamp <= key_timeout:
                            self._key_times[longitudinal] = now
            return

        toggle_debounce = float(self.get_parameter('toggle_debounce').value)
        previous = self._toggle_times.get(action, 0.0)
        if now - previous < toggle_debounce:
            return
        self._toggle_times[action] = now

        with self._lock:
            if action == 'reverse':
                self._reverse = not self._reverse
                self._gear = -1 if self._reverse else 1
                message = f"Reverse: {'ON' if self._reverse else 'OFF'}"
            elif action == 'autopilot':
                self._autopilot = not self._autopilot
                msg = Bool()
                msg.data = self._autopilot
                self._autopilot_pub.publish(msg)
                message = f"Autopilot: {'ON' if self._autopilot else 'OFF'}"
            else:
                return
        self.get_logger().info(message)

    def _open_terminal(self):
        fd = sys.stdin.fileno()
        if os.isatty(fd):
            return fd
        try:
            return os.open('/dev/tty', os.O_RDWR | os.O_NONBLOCK)
        except OSError:
            self.get_logger().error(
                'Keyboard input requires an interactive terminal. '
                'Start the container with -it.'
            )
            return None

    def _poll_terminal_input(self, now):
        if self._input_fd is None:
            return
        readable, _, _ = select.select([self._input_fd], [], [], 0)
        if not readable:
            return

        self._input_buffer += os.read(self._input_fd, 32).decode(errors='ignore')
        data = self._input_buffer
        while data:
            if data[0] == '\x1b':
                if len(data) < 3:
                    self._input_buffer = data
                    return
                key, data = data[:3], data[3:]
            else:
                key, data = data[0], data[1:]

            if key == '\x03':
                rclpy.shutdown()
                return

            action = {
                'w': 'throttle',
                's': 'brake',
                'a': 'left',
                'd': 'right',
                '\x1b[A': 'throttle', '\x1bOA': 'throttle',
                '\x1b[B': 'brake', '\x1bOB': 'brake',
                '\x1b[D': 'left', '\x1bOD': 'left',
                '\x1b[C': 'right', '\x1bOC': 'right',
                'q': 'reverse', ' ': 'handbrake', 'p': 'autopilot',
            }.get(key.lower())
            if action is not None:
                self._handle_key(action, now)
        self._input_buffer = ''

    def _publish(self):
        now = time.monotonic()
        self._poll_terminal_input(now)
        elapsed = min(now - self._last_publish, 0.1)
        self._last_publish = now
        throttle_rate = float(self.get_parameter('throttle_rate').value)
        brake_rate = float(self.get_parameter('brake_rate').value)
        steer_rate = float(self.get_parameter('steer_rate').value)
        steer_max = float(self.get_parameter('steer_max').value)
        coast_brake = float(self.get_parameter('coast_brake').value)
        longitudinal_timeout = float(
            self.get_parameter('longitudinal_key_timeout').value
        )
        steering_timeout = float(
            self.get_parameter('steering_key_timeout').value
        )

        with self._lock:
            active = {}
            for action, timestamp in self._key_times.items():
                timeout = (
                    steering_timeout
                    if action in {'left', 'right'}
                    else longitudinal_timeout
                )
                active[action] = now - timestamp <= timeout

            if active.get('brake'):
                self._brake = min(self._brake + brake_rate * elapsed, 1.0)
                self._throttle = 0.0
            elif active.get('throttle'):
                self._throttle = min(self._throttle + throttle_rate * elapsed, 1.0)
                self._brake = 0.0
            else:
                self._throttle = 0.0
                self._brake = coast_brake

            steer_increment = steer_rate * elapsed
            if active.get('left'):
                self._steer_cache = (
                    0.0 if self._steer_cache > 0
                    else self._steer_cache - steer_increment
                )
            elif active.get('right'):
                self._steer_cache = (
                    0.0 if self._steer_cache < 0
                    else self._steer_cache + steer_increment
                )
            else:
                self._steer_cache = 0.0
            self._steer_cache = min(steer_max, max(-steer_max, self._steer_cache))

            msg = CarlaEgoVehicleControl()
            msg.throttle          = float(self._throttle)
            msg.steer             = float(round(self._steer_cache, 1))
            msg.brake             = float(self._brake)
            msg.hand_brake        = active.get('handbrake', False)
            msg.reverse            = self._reverse
            msg.gear               = self._gear
            msg.manual_gear_shift  = False

        self._ctrl_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        if self._input_fd is not None:
            if self._input_old is not None:
                termios.tcsetattr(self._input_fd, termios.TCSADRAIN, self._input_old)
            if self._input_fd != sys.stdin.fileno():
                os.close(self._input_fd)
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
