#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, Float32
import serial
import threading
import math
import time

# Conversion helpers
def nmea_to_decimal(degree_min, hemi):
    if not degree_min:
        return None
    d, m = divmod(float(degree_min), 100)
    dec = d + m / 60
    if hemi in ['S', 'W']:
        dec *= -1
    return dec

def knots_to_mps(knots):
    return knots * 0.514444

class GPSNode(Node):
    def __init__(self, port="/dev/ttyAMA0", baud=9600):
        super().__init__('gps_node_filtered_logging')

        self.get_logger().info(f"Starting GPS node on {port} at {baud} baud")

        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/gps/velocity', 10)
        self.sats_pub = self.create_publisher(Int32, '/gps/satellites', 10)
        self.hdop_pub = self.create_publisher(Float32, '/gps/hdop', 10)

        # Serial port
        self.ser = serial.Serial(port, baudrate=baud, timeout=1.0)

        # Start read thread
        threading.Thread(target=self.read_loop, daemon=True).start()

    def read_loop(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
            except serial.SerialException:
                continue
            if not line:
                continue

            # --- Parse GGA for position, satellites, HDOP ---
            if 'GGA' in line:
                try:
                    parts = line.split(',')
                    lat_raw = parts[2] if len(parts) > 2 else ''
                    ns = parts[3] if len(parts) > 3 else ''
                    lon_raw = parts[4] if len(parts) > 4 else ''
                    ew = parts[5] if len(parts) > 5 else ''
                    sats = int(parts[7]) if len(parts) > 7 and parts[7].isdigit() else 0
                    hdop = float(parts[8]) if len(parts) > 8 and parts[8] else 99.9
                    alt = float(parts[9]) if len(parts) > 9 and parts[9] else 0.0

                    lat = nmea_to_decimal(lat_raw, ns)
                    lon = nmea_to_decimal(lon_raw, ew)

                    # NavSatFix
                    fix_msg = NavSatFix()
                    fix_msg.header.stamp = self.get_clock().now().to_msg()
                    fix_msg.header.frame_id = "gps"
                    fix_msg.latitude = lat if lat else 0.0
                    fix_msg.longitude = lon if lon else 0.0
                    fix_msg.altitude = alt

                    # Covariance scaled by HDOP^2, clipped
                    cov_val = (hdop if hdop > 0 else 1.0) ** 2
                    cov_val = min(cov_val, 1000.0)
                    fix_msg.position_covariance = [
                        float(cov_val), 0.0, 0.0,
                        0.0, float(cov_val), 0.0,
                        0.0, 0.0, float(cov_val)
                    ]
                    fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

                    # Publish
                    self.fix_pub.publish(fix_msg)
                    self.sats_pub.publish(Int32(data=sats))
                    self.hdop_pub.publish(Float32(data=hdop))

                    # Logging
                    # self.get_logger().info(f"Pos: {lat:.6f},{lon:.6f} Alt:{alt:.1f}m HDOP:{hdop} Sats:{sats}")

                except Exception as e:
                    self.get_logger().warn(f"GGA parse error: {e}")
                    continue

            # --- Parse RMC for speed and heading ---
            elif 'RMC' in line:
                try:
                    parts = line.split(',')
                    # speed in knots and heading in degrees
                    speed_knots = parts[7].strip() if len(parts) > 7 else '0'
                    course_deg = parts[8].strip() if len(parts) > 8 else '0'

                    # Remove any trailing non-numeric chars
                    speed_knots = ''.join(c for c in speed_knots if c in '0123456789.')
                    course_deg = ''.join(c for c in course_deg if c in '0123456789.')

                    # Convert safely
                    try:
                        speed_mps = knots_to_mps(float(speed_knots))
                        # if speed_mps < 0.1:  # filter stationary noise
                        #     speed_mps = 0.0
                    except ValueError:
                        speed_mps = 0.0

                    try:
                        heading = float(course_deg)
                    except ValueError:
                        heading = 0.0

                    vel_msg = TwistStamped()
                    vel_msg.header.stamp = self.get_clock().now().to_msg()
                    vel_msg.header.frame_id = "gps"
                    vel_msg.twist.linear.x = speed_mps * math.cos(math.radians(heading))
                    vel_msg.twist.linear.y = speed_mps * math.sin(math.radians(heading))
                    vel_msg.twist.linear.z = 0.0
                    self.vel_pub.publish(vel_msg)

                    # Logging
                    # self.get_logger().info(f"Speed: {speed_mps:.2f} m/s Heading: {heading:.1f}°")

                except Exception as e:
                    self.get_logger().warn(f"RMC parse error: {e}")
                    continue

def main():
    rclpy.init()
    node = GPSNode(port="/dev/serial0", baud=9600)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
