#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading

from sensor_msgs.msg import Range
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')

        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # Publishers
        self.alt_pub = self.create_publisher(Float32, 'barometer/altitude', 10)
        self.ultra_pub = self.create_publisher(Range, 'ultrasonic/range', 10)
        self.laser_pub = self.create_publisher(Range, 'laser/range', 10)

        # Sensor configuration
        self.sensor_cfg = {
            'G': {  # Ultrasonic
                'pub': self.ultra_pub,
                'min': 0.02,
                'max': 6.0,
                'fov': 0.6,
                'radiation': Range.ULTRASOUND
            },
            'V': {  # Laser
                'pub': self.laser_pub,
                'min': 0.02,
                'max': 1.0,
                'fov': 0.05,
                'radiation': Range.INFRARED
            }
        }

        # Serial setup
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f'Serial connected on {port}')
        except serial.SerialException as e:
            self.get_logger().fatal(str(e))
            raise

        self.running = True
        threading.Thread(target=self.read_serial, daemon=True).start()

    def read_serial(self):
        while self.running and rclpy.ok():
            try:
                line = self.ser.readline().decode().strip()
                if line:
                    self.parse_message(line)
            except UnicodeDecodeError:
                self.get_logger().warn('Foreign (non-UTF8) serial message')
            except Exception as e:
                self.get_logger().error(f'Serial error: {e}')

    def parse_message(self, line: str):
        parts = line.split(',')

        if len(parts) == 3:
            self.handle_sensor(parts, line)
        else:
            self.get_logger().warn(f'Foreign or malformed message: "{line}"')

    def handle_sensor(self, parts, line):
        device, status, value = parts

        if status != '1':
            self.get_logger().warn(f'Sensor {device} invalid data: {parts}')
            return

        if device == 'B':  # BMP Altimeter
            try:
                alt_mm = int(value)
                alt_m = alt_mm / 1000.0
                msg = Float32()
                msg.data = alt_m
                self.alt_pub.publish(msg)
            except ValueError:
                self.get_logger().warn(f'Bad altitude value: "{line}"')
            return

        if device not in self.sensor_cfg:
            self.get_logger().warn(f'Unknown device "{device}": "{line}"')
            return

        try:
            dist_m = float(value) / 1000.0
        except ValueError:
            self.get_logger().warn(f'Bad numeric value: "{line}"')
            return

        cfg = self.sensor_cfg[device]

        if dist_m < cfg['min'] or dist_m > cfg['max']:
            self.get_logger().warn(f'Out-of-range value for {device}: {dist_m:.2f} m')
            return

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{device}_frame'
        msg.radiation_type = cfg['radiation']
        msg.field_of_view = cfg['fov']
        msg.min_range = cfg['min']
        msg.max_range = cfg['max']
        msg.range = dist_m
        cfg['pub'].publish(msg)

    def destroy_node(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArduinoSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
