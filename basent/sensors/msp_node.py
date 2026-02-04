#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Joy
from std_msgs.msg import Header,Float32
from scipy.spatial.transform import Rotation
import math

from pymultiwii import MultiWii  # your multiwii.py script

class MultiWiiRosNode(Node):
    def __init__(self, port="/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0"):
        super().__init__('multiwii_node')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'multiwii/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'multiwii/mag', 10)
        self.alt_pub = self.create_publisher(Float32, 'multiwii/altitude', 10)
        self.rc_pub = self.create_publisher(Joy, 'multiwii/rc', 10)

        # Subscriber to manual RC / joystick inputs
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Connect to MultiWii flight controller
        self.get_logger().info(f"Connecting to MultiWii FC on {port}...")
        self.fc = MultiWii(port)

        # Timer for publishing FC data
        self.timer = self.create_timer(0.05, self.publish_data)  # 20 Hz

    def publish_data(self):
        try:
            imu_data = self.fc.getData(MultiWii.RAW_IMU)
            rc_data = self.fc.getData(MultiWii.RC)
            alt_data = self.fc.getData(MultiWii.ALTITUDE)
            att_data = self.fc.getData(MultiWii.ATTITUDE)

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "base_link"

            # --- IMU ---
            imu_msg = Imu()
            imu_msg.header = header
            imu_msg.linear_acceleration.x = float(imu_data['ax'])
            imu_msg.linear_acceleration.y = float(imu_data['ay'])
            imu_msg.linear_acceleration.z = float(imu_data['az'])
            imu_msg.angular_velocity.x = math.radians(float(imu_data['gx']))
            imu_msg.angular_velocity.y = math.radians(float(imu_data['gy']))
            imu_msg.angular_velocity.z = math.radians(float(imu_data['gz']))
            
             # --- Orientation (RPY → Quaternion using SciPy) ---
            roll  = math.radians(att_data['angx'])
            pitch = math.radians(att_data['angy'])
            yaw   =- math.radians(att_data['heading'])

            # ROS uses intrinsic rotations about X, Y, Z (roll, pitch, yaw)
            r = Rotation.from_euler('xyz', [roll, pitch, yaw])
            qx, qy, qz, qw = r.as_quat()  # SciPy returns [x, y, z, w]

            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            self.imu_pub.publish(imu_msg)

            # --- Magnetometer ---
            mag_msg = MagneticField()
            mag_msg.header = header
            mag_msg.magnetic_field.x = float(imu_data['mx'])
            mag_msg.magnetic_field.y = float(imu_data['my'])
            mag_msg.magnetic_field.z = float(imu_data['mz'])
            self.mag_pub.publish(mag_msg)

            # --- Pressure / Altitude ---
            alt_msg = Float32()
            alt_msg.data = float(alt_data['estalt']/100.0) #convert cm to m
            self.alt_pub.publish(alt_msg)

            # --- RC from FC ---
            joy_msg = Joy()
            joy_msg.header = header
            joy_msg.axes = [
                float(rc_data['roll']),
                float(rc_data['pitch']),
                float(rc_data['yaw']),
                float(rc_data['throttle'])
            ]
            joy_msg.buttons = []  # optional: map switches if available
            self.rc_pub.publish(joy_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to read or publish data: {e}")

    def joy_callback(self, msg: Joy):
        """
        Receive joystick or manual RC commands and forward to MultiWii.
        Assumes axes: [roll, pitch, yaw, throttle]
        """
        try:
            if len(msg.axes) < 4:
                self.get_logger().warn("Joy message does not have enough axes")
                return

            rc_values = [
                int(msg.axes[0]),  # roll
                int(msg.axes[1]),  # pitch
                int(msg.axes[2]),  # yaw
                int(msg.axes[3])   # throttle
            ]

            # Send to FC
            self.fc.sendCMD(8, MultiWii.SET_RAW_RC, rc_values)
        except Exception as e:
            self.get_logger().error(f"Failed to send RC values to MultiWii: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiWiiRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down MultiWii ROS node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
