#!/usr/bin/env python3

import csv
import math
import os
import board
import busio

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


class CV0850Device:

    ADDR = 0x31

    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        while not self.i2c.try_lock():
            pass

    def read_u8(self, reg):
        buf = bytearray(1)
        self.i2c.writeto_then_readfrom(self.ADDR, bytes([reg]), buf)
        return buf[0]

    def read_s16(self, reg):
        buf = bytearray(2)
        self.i2c.writeto_then_readfrom(self.ADDR, bytes([reg]), buf)
        v = buf[0] | (buf[1] << 8)
        if v >= 0x8000:
            v -= 0x10000
        return v

    def read_u16(self, reg):
        buf = bytearray(2)
        self.i2c.writeto_then_readfrom(self.ADDR, bytes([reg]), buf)
        return buf[0] | (buf[1] << 8)


class CV0850(Node, CV0850Device):

    # ---------------- CONFIG ----------------
    X_AXIS = "y"   # options: "x", "y"
    Y_AXIS = "x"   # options: "x", "y"

    INVERT_X = -1   # -1 flips direction
    INVERT_Y = -1

    USE_TOF_LIMIT = 7.5  # m

    def __init__(self):
        Node.__init__(self, "cv0850_oriented")
        CV0850Device.__init__(self)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().info("Optical flow node initialized")

        # ROS
        self.create_subscription(Imu, "multiwii/imu", self.imu_cb, 10)
        self.create_subscription(Float32, "multiwii/altitude", self.alt_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, "optical_flow/odom", 10)

        # state
        self.orientation = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = None
        self.yaw_intial = None

        self.height_m = 0.3

        self.x = 0.0
        self.y = 0.0

        self.vx_f = 0.0
        self.vy_f = 0.0
        self.last_update_time = None

        self.csv_path = os.path.join(os.getcwd(), "optical_flow_data.csv")
        self._init_csv()

        self.create_timer(0.01, self.update)

    # ---------------- CSV logging ----------------

    def _init_csv(self):
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["dt", "accumulated_time", "flow_x", "flow_y", "tof_strength", "x", "y", "roll", "pitch", "yaw", "altitude", "tof_h", "baro_h"])
        self.get_logger().info(f"CSV logging initialized at {self.csv_path}")

    def _log_csv(self, dt, accumulated_time, flow_x, flow_y, tof_strength, x, y, roll, pitch, yaw, altitude, tof_h, baro_h):
        with open(self.csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([dt, accumulated_time, flow_x, flow_y, tof_strength, x, y, roll, pitch, yaw, altitude, tof_h, baro_h])

    # ---------------- callbacks ----------------

    def imu_cb(self, msg):
        self.orientation = msg.orientation
        q = msg.orientation
        self.get_logger().debug(
            f"IMU callback: yaw={quat_to_yaw(q):.3f} roll={math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y)):.3f} pitch={math.asin(2*(q.w*q.y - q.z*q.x)):.3f}"
        )

        if self.yaw_intial is None:
            self.yaw_intial = quat_to_yaw(q)
        
        self.yaw = quat_to_yaw(q) - self.yaw_intial

        # roll
        self.roll = math.atan2(
            2*(q.w*q.x + q.y*q.z),
            1 - 2*(q.x*q.x + q.y*q.y)
        )

        # pitch
        self.pitch = math.asin(
            2*(q.w*q.y - q.z*q.x)
        )

    def alt_cb(self, msg):
        self.height_m = msg.data*10
        self.get_logger().debug(f"Altitude callback: height_m={self.height_m:.3f}")

    # ---------------- axis mapping ----------------

    def map_axes(self, dx, dy):

        if self.X_AXIS == "x":
            mx = dx
        else:
            mx = dy

        if self.Y_AXIS == "y":
            my = dy
        else:
            my = dx

        return self.INVERT_X * mx, self.INVERT_Y*my

    # ---------------- update ----------------

    def update(self):

        if self.orientation is None:
            self.get_logger().debug(
                f"Orientation is none"
            )
            return

        try:
            if self.read_u8(0x0B) != 245:
                # self.get_logger().warn(
                #     f"CV0850 not ready, check wiring and power: {self.read_u8(0x0B)}"
                # )
                return

            flow_x = self.read_s16(0x05)
            flow_y = self.read_s16(0x07)
            
            # Read accumulated time (integration_timespan) from registers 0x09-0x0A (in microseconds)
            accumulated_time_us = self.read_u16(0x09)
            accumulated_time_s = accumulated_time_us / 1e6  # Convert to seconds
            
            # Read TOF strength
            tof_strength = self.read_u8(0x03)
            
            self.get_logger().debug(
                f"Raw flow values: flow_x={flow_x} flow_y={flow_y} accumulated_time_us={accumulated_time_us} tof_strength={tof_strength}"
            )

            # ToF + baro height (simple)
            tof_m = self.read_u16(0x01) / 1000.0
            tof_h = tof_m * math.cos(self.roll) * math.cos(self.pitch)

            h = tof_h #if tof_m < self.USE_TOF_LIMIT else self.height_m


            # CV0850 model - displacement calculation using accumulated time
            h_mm = h * 1000.0

            # flow_x and flow_y are in radians*10000, accumulated over accumulated_time
            # displacement = (flow_integral / 10000) * height_mm
            dx = (flow_x / 10000.0) * h_mm / 1000.0  # Convert back to meters
            dy = (flow_y / 10000.0) * h_mm / 1000.0

            # axis remap (THIS IS YOUR NEW FEATURE)
            dx, dy = self.map_axes(dx, dy)

            # world frame
            #wx = math.cos(self.yaw) * dx - math.sin(self.yaw) * dy
            #wy = math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
            wx=dx
            wy=dy
            # velocity using accumulated time from sensor (in seconds)
            dt = accumulated_time_s

            vx = wx / accumulated_time_s
            vy = wy / accumulated_time_s

            # filter
            alpha = 0.25
            self.vx_f = alpha * vx + (1 - alpha) * self.vx_f
            self.vy_f = alpha * vy + (1 - alpha) * self.vy_f

            # deadband
            if abs(self.vx_f) < 0.01:
                self.vx_f = 0.0
            if abs(self.vy_f) < 0.01:
                self.vy_f = 0.0

            # integrate (for RViz guide only)
            self.x += self.vx_f * dt
            self.y += self.vy_f * dt

            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = h

            odom.pose.pose.orientation = self.orientation

            odom.twist.twist.linear.x = self.vx_f
            odom.twist.twist.linear.y = self.vy_f

            self.odom_pub.publish(odom)
            if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
                self._log_csv(dt, accumulated_time_s, flow_x, flow_y, tof_strength, self.x, self.y, self.roll, self.pitch, self.yaw, h, tof_h, self.height_m)

            self.get_logger().debug(
                f"Publishing odom: x={self.x:.2f} y={self.y:.2f} h={h:.2f} vx={self.vx_f:.3f} vy={self.vy_f:.3f} roll={self.roll:.2f} pitch={self.pitch:.2f} yaw={self.yaw:.2f}"
            )
            self.get_logger().debug(
                f"Height breakdown: tof_h={tof_h:.2f} tof_m={tof_m:.2f} tof_strength={tof_strength} baro_h={self.height_m:.2f} flow_integrals: x={flow_x} y={flow_y}"
            )

        except Exception as e:
            self.get_logger().warn(str(e))


def main():
    rclpy.init()
    node = CV0850()
    rclpy.spin(node)

    node.i2c.unlock()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()