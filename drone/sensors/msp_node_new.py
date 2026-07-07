#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, Joy
from std_msgs.msg import Float32, String, Header

from scipy.spatial.transform import Rotation
import threading
from yamspy import MSPy


class MultiWiiRosNode(Node):
    def __init__(self):
        super().__init__('multiwii_node')

        self.declare_parameter(
            'port', '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0'
        )
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('startup_wait_s', 5.0)
        self.declare_parameter('arm_aux_value', 2000)
        self.declare_parameter('disarm_aux_value', 1000)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.startup_wait_s = float(self.get_parameter('startup_wait_s').value)
        self.arm_aux_value = int(self.get_parameter('arm_aux_value').value)
        self.disarm_aux_value = int(self.get_parameter('disarm_aux_value').value)

        self.imu_pub = self.create_publisher(Imu, 'multiwii/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'multiwii/mag', 10)
        self.alt_pub = self.create_publisher(Float32, 'multiwii/altitude', 10)
        self.voltage_pub = self.create_publisher(Float32, 'multiwii/voltage', 10)
        self.rc_pub = self.create_publisher(Joy, 'multiwii/rc_out', 10)

        self.create_subscription(Joy, 'multiwii/rc_in', self.joy_callback, 10)
        self.create_subscription(String, '/mission/state', self.mission_cb, 10)

        self.armed = False
        self.last_arm_state = None
        self.last_rc_frame = [1500, 1500, 1000, 1500, self.disarm_aux_value, 1000, 1000, 1000]
        self.pending_rc_frame = None
        self.last_rc_sent_time = 0.0
        self.rc_lock = threading.RLock()

        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.get_logger().info(f'Connecting to FC on {self.port} @ {self.baudrate}...')
        self.fc = MSPy(device=self.port, baudrate=self.baudrate, loglevel='WARNING')
        self.fc_lock = threading.RLock()
        self.fc_connected = False
        self.connect_fc()

        time.sleep(self.startup_wait_s)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_data)

        self.rc_sender_thread = threading.Thread(target=self._rc_sender_loop, daemon=True)
        self.rc_sender_thread.start()
        self.get_logger().debug('RC sender thread started with 0.1s interval')


    def connect_fc(self):
        self.get_logger().debug('connect_fc: enter')
        with self.fc_lock:
            if getattr(self.fc, 'conn', None) is not None and getattr(self.fc.conn, 'is_open', False):
                self.fc_connected = True
                self.get_logger().debug('connect_fc: already connected')
                return True
            try:
                result = self.fc.connect(trials=3, delay=0.5)
                self.get_logger().debug(f'connect_fc: connect result={result}')
                if result == 0:
                    if self.fc.conn:
                        self.fc.conn.timeout = 0.1  # set once here, not per read/write
                    self.fc_connected = True
                    self.get_logger().info(f'Connected to flight controller on {self.port}')
                    return True
            except Exception as e:
                self.get_logger().warn(f'FC connection attempt failed: {e}')

            self.fc_connected = False
            self.get_logger().warn(
                f'Flight controller not available on {self.port}; telemetry will be skipped until it reconnects'
            )
            self.get_logger().debug('connect_fc: exit false')
            return False


    def _read_fast_data(self):
        self.get_logger().debug('read_fast_data: enter')
        if not self.fc_connected:
            self.get_logger().debug('read_fast_data: fc not connected, reconnecting')
            if not self.connect_fc():
                self.get_logger().debug('read_fast_data: reconnect failed')
                return None

        with self.fc_lock:
            try:
                self.get_logger().debug('read_fast_data: before fast_read_imu')
                self.fc.fast_read_imu()
                self.get_logger().debug('read_fast_data: before fast_read_attitude')
                self.fc.fast_read_attitude()
                self.get_logger().debug('read_fast_data: before fast_read_altitude')
                self.fc.fast_read_altitude()
                self.get_logger().debug('read_fast_data: before fast_read_analog')
                self.fc.fast_read_analog()
                self.get_logger().debug('read_fast_data: success')
                return {
                    'sensor_data': self.fc.SENSOR_DATA,
                    'altitude': self.fc.SENSOR_DATA['altitude'],
                    'analog': self.fc.ANALOG,
                }
            except Exception as e:
                self.get_logger().warn(f'Timeout reading telemetry: {e}')
                self.fc_connected = False  # force a reconnect+re-validate next call
                self.get_logger().debug('read_fast_data: exception, fc_connected false')
                return None

    def _write_rc_frame(self, frame):
        self.get_logger().debug(f'write_rc_frame: enter {frame}')
        if not self.fc_connected:
            self.get_logger().debug('write_rc_frame: fc not connected, reconnecting')
            if not self.connect_fc():
                self.get_logger().debug('write_rc_frame: reconnect failed')
                return False

        with self.fc_lock:
            try:
                self.get_logger().debug(f'write_rc_frame: sending {frame}')
                self.fc.fast_msp_rc_cmd(frame)
                self.last_rc_sent_time = time.monotonic()
                self.last_rc_frame = frame
                self.get_logger().debug('write_rc_frame: send success')
                return True
            except Exception as e:
                self.get_logger().warn(f'Failed to send RC frame: {e}')
                self.fc_connected = False
                self.get_logger().debug('write_rc_frame: exception, fc_connected false')
                return False

    def _queue_rc_frame(self, roll, pitch, throttle, yaw, arm=False):
        aux1 = self.arm_aux_value if arm else self.disarm_aux_value
        frame = [int(roll), int(pitch), int(throttle), int(yaw), aux1, 1000, 1000, 1000]
        with self.rc_lock:
            self.pending_rc_frame = frame
        self.get_logger().debug(f'queue_rc_frame: queued {frame}')
        return frame

    def _rc_sender_loop(self):
        self.get_logger().debug('rc_sender_loop: started')
        next_send = time.monotonic()
        while rclpy.ok():
            with self.rc_lock:
                if self.pending_rc_frame is not None:
                    frame = self.pending_rc_frame
                    self.pending_rc_frame = None
                else:
                    frame = self.last_rc_frame

            if frame is None:
                frame = [1500, 1500, 1000, 1500, self.disarm_aux_value, 1000, 1000, 1000]
                self.get_logger().debug(f'rc_sender_loop: default frame {frame}')

            sent = self._write_rc_frame(frame)
            self.get_logger().debug(f'rc_sender_loop: sent frame={frame} success={sent}')

            next_send += 0.1
            sleep_time = next_send - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_send = time.monotonic()

    def publish_data(self):
        self.get_logger().debug('publish_data: enter')
        try:
            snap = self._read_fast_data()
            if not snap:
                self.get_logger().debug('publish_data: no snapshot')
                return

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'base_link'

            sensor_data = snap['sensor_data']
            altitude = snap['altitude']
            analog = snap['analog']

            imu_msg = Imu()
            imu_msg.header = header

            accel = sensor_data['accelerometer']
            gyro = sensor_data['gyroscope']
            mag = sensor_data['magnetometer']
            kinematics = sensor_data['kinematics']

            if not accel or not gyro or not mag or not kinematics:
                self.get_logger().warn('Flight controller returned incomplete IMU data')
                return

            imu_msg.linear_acceleration.x = float(accel[0])
            imu_msg.linear_acceleration.y = float(accel[1])
            imu_msg.linear_acceleration.z = float(accel[2])
            imu_msg.angular_velocity.x = math.radians(float(gyro[0]))
            imu_msg.angular_velocity.y = math.radians(float(gyro[1]))
            imu_msg.angular_velocity.z = math.radians(float(gyro[2]))

            roll = math.radians(float(kinematics[0]))
            pitch = math.radians(float(kinematics[1]))
            yaw = -math.radians(float(kinematics[2]))
            qx, qy, qz, qw = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw

            imu_msg.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
            imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            imu_msg.linear_acceleration_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
            self.imu_pub.publish(imu_msg)

            mag_msg = MagneticField()
            mag_msg.header = header
            mag_msg.magnetic_field.x = float(mag[0])
            mag_msg.magnetic_field.y = float(mag[1])
            mag_msg.magnetic_field.z = float(mag[2])
            self.mag_pub.publish(mag_msg)

            alt_msg = Float32()
            alt_msg.data = float(altitude) / 100.0
            self.alt_pub.publish(alt_msg)

            if 'voltage' in analog:
                volt_msg = Float32()
                volt_msg.data = float(analog['voltage'])
                self.voltage_pub.publish(volt_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to read or publish data: {e}')

    def _send_raw_rc(self, roll=1500, pitch=1500, throttle=1000, yaw=1500, arm=False):
        self.get_logger().debug(f'send_raw_rc: enter roll={roll} pitch={pitch} throttle={throttle} yaw={yaw} arm={arm}')
        return self._queue_rc_frame(roll, pitch, throttle, yaw, arm)

    def joy_callback(self, msg: Joy):
        self.get_logger().debug(f'joy_callback: enter axes={msg.axes}')
        try:
            if len(msg.axes) < 4:
                self.get_logger().warn('Joy message does not have enough axes')
                return
            roll = msg.axes[0]
            pitch = msg.axes[1]
            throttle = msg.axes[3]
            yaw = msg.axes[2]
            frame = self._queue_rc_frame(roll, pitch, throttle, yaw, self.armed)
            self.get_logger().info(f'Queued Joy frame for RC send: {frame} armed={self.armed}')
            self.get_logger().debug('joy_callback: exit')
        except Exception as e:
            self.get_logger().error(f'Failed to send RC values to FC: {e}')

    def mission_cb(self, msg: String):
        state = msg.data.strip().lower()
        self.get_logger().debug(f'mission_cb: enter state={state}')
        try:
            if state == 'manual':
                if not self.armed:
                    self.get_logger().info('Manual state received: arming via AUX1')
                    self.armed = True
                    frame = self._queue_rc_frame(1500, 1500, 1000, 1500, True)
                    self.get_logger().info(f'Queued RC arm frame: {frame}')
            elif state == 'disarm':
                self.get_logger().info('Disarm state received: disarming via AUX1')
                frame = self._queue_rc_frame(1500, 1500, 1000, 1500, False)
                self.get_logger().info(f'Queued RC disarm frame: {frame}')
                self.armed = False
            elif state == 'pos_hold':
                self.get_logger().debug('pos_hold received: no arm change')
            self.get_logger().debug('mission_cb: exit')
        except Exception as e:
            self.get_logger().error(f'Mission state handling failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MultiWiiRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down MultiWii ROS node...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()