"""
joystick_teleop: FlySky FS-i6X (Mode 2) -> /cmd_vel

Normalised stick convention (after optional inversion):
    roll     right   = +   (right stick horizontal, CH1)
    pitch    forward = +   (right stick vertical,   CH2)
    throttle up      = +   (left stick vertical,    CH3)
    yaw      right   = +   (left stick horizontal,  CH4)

Gestures (left stick, right stick centred, held arm_hold_time seconds):
    ARM    : throttle <= -T and yaw <= -T  (bottom-left)  -> angular.x =  2.0
    DISARM : throttle <= -T and yaw >= +T  (bottom-right) -> angular.x = -2.0

The FS-i6X throttle stick does NOT spring back to centre. After arming it is
still at the bottom, which as a climb-rate command would mean "full descend".
So throttle is ignored after arming until the stick has passed through mid.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

ARM_CMD = 2.0
DISARM_CMD = -2.0


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        p = self.declare_parameter
        p('axis_roll', 0)
        p('axis_pitch', 1)
        p('axis_throttle', 2)
        p('axis_yaw', 3)
        p('invert_roll', False)
        p('invert_pitch', False)
        p('invert_throttle', False)
        p('invert_yaw', False)
        p('scale_linear_x', 2.0)
        p('scale_linear_y', 2.0)
        p('scale_linear_z', 1.5)
        p('scale_angular_z', 1.5)
        p('stick_deadzone', 0.08)
        p('stick_threshold', 0.75)
        p('center_threshold', 0.3)
        p('arm_hold_time', 0.5)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.ax = {k: int(g(f'axis_{k}')) for k in ('roll', 'pitch', 'throttle', 'yaw')}
        self.inv = {k: -1.0 if g(f'invert_{k}') else 1.0 for k in self.ax}
        self.sx = float(g('scale_linear_x'))
        self.sy = float(g('scale_linear_y'))
        self.sz = float(g('scale_linear_z'))
        self.saz = float(g('scale_angular_z'))
        self.dz = float(g('stick_deadzone'))
        self.thr = float(g('stick_threshold'))
        self.ctr = float(g('center_threshold'))
        self.hold = float(g('arm_hold_time'))

        self.armed = False
        self.throttle_live = False      # becomes True once throttle passes mid after arming
        self.gesture = None             # 'arm' | 'disarm' | None
        self.gesture_start = None
        self.gesture_fired = False      # must release before the same gesture fires again
        self.warned_axes = False

        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            f"FS-i6X teleop ready. axes={self.ax} inverted="
            f"{[k for k, v in self.inv.items() if v < 0]}")

    def apply_deadzone(self, v):
        if abs(v) < self.dz:
            return 0.0
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - self.dz) / (1.0 - self.dz)

    def joy_callback(self, msg):
        need = max(self.ax.values()) + 1
        if len(msg.axes) < need:
            if not self.warned_axes:
                self.get_logger().error(
                    f"/joy has {len(msg.axes)} axes, need {need}. Check axis_* args.")
                self.warned_axes = True
            return

        raw = {k: self.inv[k] * msg.axes[i] for k, i in self.ax.items()}
        roll, pitch, throttle, yaw = raw['roll'], raw['pitch'], raw['throttle'], raw['yaw']

        # ---------------- gesture detection ----------------
        right_centred = abs(roll) < self.ctr and abs(pitch) < self.ctr
        thr_low = throttle <= -self.thr
        current = None
        if right_centred and thr_low:
            if yaw <= -self.thr:
                current = 'arm'
            elif yaw >= self.thr:
                current = 'disarm'

        now = self.get_clock().now()
        if current != self.gesture:
            self.gesture = current
            self.gesture_start = now
            self.gesture_fired = False
        elif current is not None and not self.gesture_fired:
            held = (now - self.gesture_start).nanoseconds / 1e9
            if held >= self.hold:
                self.gesture_fired = True
                self.fire(current)
                return

        # ---------------- stick -> velocity ----------------
        twist = Twist()
        if self.armed:
            if not self.throttle_live and abs(throttle) < 0.15:
                self.throttle_live = True
                self.get_logger().info("Throttle centred - altitude control live.")

            twist.linear.x = self.apply_deadzone(pitch) * self.sx
            twist.linear.y = -self.apply_deadzone(roll) * self.sy
            if self.throttle_live:
                twist.linear.z = self.apply_deadzone(throttle) * self.sz
            # Don't yaw while the operator is holding a gesture
            if current is None:
                twist.angular.z = -self.apply_deadzone(yaw) * self.saz
        self.pub.publish(twist)

    def fire(self, gesture):
        t = Twist()
        if gesture == 'arm':
            t.angular.x = ARM_CMD
            self.armed = True
            self.throttle_live = False
            self.get_logger().warn("ARM gesture -> OFFBOARD + ARM. Bring throttle to MID.")
        else:
            t.angular.x = DISARM_CMD
            self.armed = False
            self.throttle_live = False
            self.get_logger().warn("DISARM gesture -> DISARM (PX4 rejects it in the air).")
        self.pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
