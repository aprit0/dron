#!/usr/bin/env python3

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry


class RCController(Node):

    def __init__(self):
        super().__init__("rc_controller")
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("RC controller node initialized")

        # ---------------- subscriptions ----------------
        self.create_subscription(Odometry,
                                 "optical_flow/odom",
                                 self.odom_cb,
                                 10)

        self.create_subscription(Joy,
                                 "multiwii/rc_in",
                                 self.rc_cb,
                                 10)

        self.create_subscription(Joy,
                                 "/joy",
                                 self.joy_cb,
                                 10)

        self.create_subscription(String,
                                 "/mission/state",
                                 self.mission_cb,
                                 10)

        self.create_subscription(Float32,
                                 "multiwii/altitude",
                                 self.altitude_cb,
                                 10)

        # ---------------- output ----------------
        self.rc_pub = self.create_publisher(Joy,
                                            "multiwii/rc_in",
                                            10)

        # ---------------- state ----------------
        self.state = "pos_hold"

        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_altitude = 0.0
        self.hold_active = False
        self.altitude = 0.0

        self.joy_override = False
        self.internal_rc_marker = "run_alt_internal"

        # RC values (PWM style)
        self.rc_roll = 1500
        self.rc_pitch = 1500
        self.rc_yaw = 1500
        self.rc_throttle = 1000

        self.last_throttle = None

        self.create_timer(0.1, self.update)  # 50 Hz

    # ---------------- callbacks ----------------

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.get_logger().debug(
            f"Odometry updated: x={self.x:.2f} y={self.y:.2f} vx={self.vx:.2f} vy={self.vy:.2f}"
        )

    def _is_internal_rc_message(self, msg):
        return getattr(msg.header, "frame_id", "") == self.internal_rc_marker

    def rc_cb(self, msg):
        if len(msg.axes) > 3:
            if self._is_internal_rc_message(msg):
                self.get_logger().debug("Ignoring internally published RC message")
                return

            self.joy_override = True
            self.state = "manual"

            self.rc_roll = self.clamp_rc(msg.axes[0])
            self.rc_pitch = self.clamp_rc(msg.axes[1])
            self.rc_yaw = self.clamp_rc(msg.axes[2])
            self.rc_throttle = self.clamp_rc(msg.axes[3])
            self.last_throttle = int(msg.axes[3])

            self.get_logger().info(
                f"Received manual RC input: roll={self.rc_roll} pitch={self.rc_pitch} yaw={self.rc_yaw} throttle={self.rc_throttle}"
            )

    def joy_cb(self, msg):
        self.joy_override = True
        self.get_logger().warning("Joystick override activated")

    def altitude_cb(self, msg):
        self.altitude = float(msg.data)
        self.get_logger().debug(f"Altitude updated: {self.altitude:.2f}")

    def mission_cb(self, msg):
        state = msg.data.lower()

        if state != self.state:
            self.get_logger().info(f"Mission state changed from {self.state} to {state}")
            if state == "pos_hold":
                self.hold_x = self.x
                self.hold_y = self.y
                self.hold_altitude = self.altitude
                self.hold_active = True
                self.joy_override = False
                self.get_logger().info(
                    f"Position hold armed at x={self.hold_x:.2f} y={self.hold_y:.2f} altitude={self.hold_altitude:.2f}"
                )
            else:
                self.hold_active = False

        self.state = state
        

    # ---------------- control ----------------

    def clamp_rc(self, v):
        return max(1000, min(2000, int(v)))

    def update(self):
        try:
            self.get_logger().info(
                f"Update loop: state={self.state} joy_override={self.joy_override} hold_active={self.hold_active}"
            )

            # ---------------- OVERRIDE ----------------
            if self.joy_override or self.state == "manual":
                # self.get_logger().debug(
                #     f"Manual override RC: roll={self.rc_roll} pitch={self.rc_pitch} yaw={self.rc_yaw} throttle={self.rc_throttle}"
                # )
                return

            # ---------------- POSITION HOLD ----------------
            if self.state == "pos_hold":

                if not self.hold_active:
                    self.hold_x = self.x
                    self.hold_y = self.y
                    self.hold_altitude = self.altitude
                    self.hold_active = True

                # simple P control (tune later)
                k_p = 120.0
                k_p_alt = 80.0

                # use the hold setpoints captured when entering position hold
                x_error = self.hold_x - self.x
                y_error = self.hold_y - self.y
                altitude_error = self.hold_altitude - self.altitude

                # error → RC correction around 1500
                roll_cmd  = 1500 + (-y_error * k_p)
                pitch_cmd = 1500 + (-x_error * k_p)

                if self.last_throttle is None:
                    self.get_logger().warn("No throttle value received yet; waiting for initial RC input")
                    return

                throttle_cmd = self.last_throttle + (altitude_error * k_p_alt)

                # clamp
                self.rc_roll = self.clamp_rc(roll_cmd)
                self.rc_pitch = self.clamp_rc(pitch_cmd)
                self.rc_throttle = self.clamp_rc(throttle_cmd)
                self.get_logger().debug(
                    f"Position hold commands: roll={self.rc_roll} pitch={self.rc_pitch} throttle={self.rc_throttle}"
                )
                self.get_logger().info(
                    f"Position: {self.x:.2f}, {self.y:.2f} | Hold: {self.hold_x:.2f}, {self.hold_y:.2f} | Errors: x={x_error:.2f}, y={y_error:.2f}, alt={altitude_error:.2f}"
                )

                # yaw neutral
                self.rc_yaw = 1500

            # ---------------- ESTOP ----------------
            elif self.state == "estop":

                self.rc_roll = 1500
                self.rc_pitch = 1500
                self.rc_yaw = 1500
                self.rc_throttle = 1000  # cut throttle
                self.get_logger().warning("ESTOP active, cutting throttle")

            # ---------------- publish ----------------
            out = Joy()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.internal_rc_marker
            out.axes = [
                float(self.rc_roll),
                float(self.rc_pitch),
                float(self.rc_yaw),
                float(self.rc_throttle)
            ]
            self.rc_pub.publish(out)
            self.get_logger().info(
                f"Published RC output: roll={self.rc_roll} pitch={self.rc_pitch} yaw={self.rc_yaw} throttle={self.rc_throttle}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Error in update loop: {e} | joy_override={self.joy_override} state={self.state} hold_active={self.hold_active} x={self.x:.2f} y={self.y:.2f} vx={self.vx:.2f} vy={self.vy:.2f}"
            )
            self.get_logger().error(
                f"Variables: last_throttle={self.last_throttle} rc_roll={self.rc_roll} rc_pitch={self.rc_pitch} rc_yaw={self.rc_yaw} rc_throttle={self.rc_throttle}"
            )

def main():
    rclpy.init()
    node = RCController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()