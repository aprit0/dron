#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, Float32
import threading
import time
import os

class GPSCLI(Node):
    def __init__(self):
        super().__init__("gps_cli_gui")

        # Latest state storage
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.speed = None
        self.heading = None
        self.sats = None
        self.hdop = None

        # Subscribers
        self.create_subscription(NavSatFix, '/gps/fix', self.fix_cb, 10)
        self.create_subscription(TwistStamped, '/gps/velocity', self.vel_cb, 10)
        self.create_subscription(Int32, '/gps/satellites', self.sats_cb, 10)
        self.create_subscription(Float32, '/gps/hdop', self.hdop_cb, 10)

        # Start display thread
        threading.Thread(target=self.display_loop, daemon=True).start()

    def fix_cb(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude

    def vel_cb(self, msg):
        self.speed = msg.twist.linear.x
        if msg.twist.linear.x != 0 or msg.twist.linear.y != 0:
            import math
            self.heading = math.degrees(math.atan2(msg.twist.linear.y, msg.twist.linear.x))

    def sats_cb(self, msg):
        self.sats = msg.data

    def hdop_cb(self, msg):
        self.hdop = msg.data

    def display_loop(self):
        while rclpy.ok():
            os.system('clear')  # clear terminal
            print("=== GPS CLI Visualizer ===")
            print(f"Latitude : {self.latitude if self.latitude is not None else '---'}")
            print(f"Longitude: {self.longitude if self.longitude is not None else '---'}")
            print(f"Altitude : {self.altitude if self.altitude is not None else '---'} m")
            print(f"Speed    : {self.speed if self.speed is not None else '---'} m/s")
            print(f"Heading  : {self.heading if self.heading is not None else '---'}°")
            print(f"Satellites: {self.sats if self.sats is not None else '---'}")
            print(f"HDOP     : {self.hdop if self.hdop is not None else '---'}")
            print("\nPress Ctrl+C to exit")
            time.sleep(0.5)

def main():
    rclpy.init()
    node = GPSCLI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
