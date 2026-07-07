#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Joy
from std_msgs.msg import Header, Float32, String
from scipy.spatial.transform import Rotation
import math, time

from pymultiwii import MultiWii  # your multiwii.py script

class MultiWiiRosNode(Node):
    def __init__(self, port="/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0"):
        super().__init__('multiwii_node')

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'multiwii/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'multiwii/mag', 10)
        self.alt_pub = self.create_publisher(Float32, 'multiwii/altitude', 10)
        self.rc_pub = self.create_publisher(Joy, 'multiwii/rc_out', 10)

        # Subscriber to manual RC / joystick inputs
        self.create_subscription(Joy, 'multiwii/rc_in', self.joy_callback, 10)

        # Connect to MultiWii flight controller
        self.get_logger().info(f"Connecting to MultiWii FC on {port}...")
        self.fc = MultiWii(port)

        # Mission state subscriber and diagnostic publisher
        self.state_sub = self.create_subscription(String, '/mission/state', self.mission_cb, 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.armed = False

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
            ACC_SCALE = 9.80665 / 505.0   # MultiWii accel scaling
            GYRO_SCALE = math.radians(1.0 / 16.4)
            imu_msg = Imu()
            imu_msg.header = header
            if imu_data is not None:
                imu_msg.linear_acceleration.x = float(imu_data['ax']) * ACC_SCALE if abs(float(imu_data['ax']) * ACC_SCALE) > 0.8 else 0.0
                imu_msg.linear_acceleration.y = float(imu_data['ay']) * ACC_SCALE if abs(float(imu_data['ay']) * ACC_SCALE) > 0.8 else 0.0
                imu_msg.linear_acceleration.z = float(imu_data['az']) * ACC_SCALE if abs(float(imu_data['az']) * ACC_SCALE) > 0.8 else 0.0
                imu_msg.angular_velocity.x = float(imu_data['gx']) * GYRO_SCALE
                imu_msg.angular_velocity.y = float(imu_data['gy']) * GYRO_SCALE
                imu_msg.angular_velocity.z = float(imu_data['gz']) * GYRO_SCALE
                
            imu_msg.orientation_covariance = [
                0.001, 0.0, 0.0,
                0.0, 0.001, 0.0,
                0.0, 0.0, 0.001
            ]

            imu_msg.angular_velocity_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01
            ]

            imu_msg.linear_acceleration_covariance = [
                0.05, 0.0, 0.0,
                0.0, 0.05, 0.0,
                0.0, 0.0, 0.05
            ]
            # print(f'raw linear: {[imu_data["ax"], imu_data["ay"], imu_data["az"]]} raw angular: {[imu_data["gx"], imu_data["gy"], imu_data["gz"]]}')
            # print(f'scaled linear: {[round(imu_msg.linear_acceleration.x, 2), round(imu_msg.linear_acceleration.y, 2), round(imu_msg.linear_acceleration.z, 2)]} scaled angular: {[round(imu_msg.angular_velocity.x, 2), round(imu_msg.angular_velocity.y, 2), round(imu_msg.angular_velocity.z, 2)]}')
            
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
            if imu_data is not None:
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
            self.get_logger().error(f"Failed to read or publish data: {e}\n {imu_data}\n {rc_data}\n {alt_data}\n {att_data}")

    def joy_callback(self, msg: Joy):
        """
        Receive joystick or manual RC commands and forward to MultiWii.
        Assumes axes: [roll, pitch, yaw, throttle]
        """
        try:
            print(f"Received RC input: {list(msg.axes[:4])}")
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
            self.fc.sendCMD(8, MultiWii.SET_RAW_RC, rc_values, '4H')
        except Exception as e:
            self.get_logger().error(f"Failed to send RC values to MultiWii: {e}")

    def mission_cb(self, msg: String):
        state = msg.data.strip().lower()
        if state == 'manual':
            if not self.armed:
                self.get_logger().info('Mission state manual received: arming FC')
                try:
                    self.arm()
                    self.armed = True
                except Exception as e:
                    self.get_logger().error(f'Failed to arm FC: {e}')
            else:
                self.get_logger().debug('Mission state manual received, FC already armed')
        elif state == 'disarm':
            self.get_logger().info('Mission state disarm received: disarming FC')
            try:
                self.disarm()
            except Exception as e:
                self.get_logger().error(f'Failed to disarm FC: {e}')
            self.armed = False
        elif state == 'pos_hold':
            self.get_logger().debug('Mission state pos_hold received: no arm/disarm action')
        else:
            self.get_logger().debug(f'Unknown mission state received: {state}')
    
    def arm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,2000,1000]
            self.fc.sendCMD(8,MultiWii.SET_RAW_RC,data, '4H')
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()

    def disarm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,1000,1000]
            self.fc.sendCMD(8,MultiWii.SET_RAW_RC,data, '4H')
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()


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
