#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
import time
import math

class PS5ControllerNode(Node):
    def __init__(self):
        super().__init__('ps5_controller_node')

        # Publishers
        self.manual_rc_pub = self.create_publisher(Joy, '/manual/rc_out', 10)

        # Subscribe to standard joy topic (assumes joy_node running)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Processing parameters
        self.deadzone = 0.15
        self.smoothing_alpha = 0.4
        self.max_rate_hz = 50.0
        self.dt_max = 1.0 / self.max_rate_hz

        # Smoothed values
        self.smoothed_axes = {
            'roll': 1500.0,
            'pitch': 1500.0,
            'yaw': 1500.0,
            'throttle': 1500.0
        }

        # Rate limiting
        self.last_publish_time = 0.0

        self.get_logger().info('PS5 Controller Node initialized. Deadzone: {:.2f}, Smoothing: {:.2f}, Rate: {}Hz'.format(
            self.deadzone, self.smoothing_alpha, self.max_rate_hz))

    def apply_deadzone(self, value, dz):
        if abs(value) < dz:
            return 0.0
        return value

    def joy_to_pwm(self, value, is_throttle=False):
        if value == 0.0:
            return 1500.0
        if is_throttle:
            # Throttle: stick up (-1 on Y) -> high throttle (2000), down (+1) -> low (1000)
            return 1500.0 - value * 500.0  # axes[1]=-1 -> 2000; +1 -> 1000
        else:
            return 1500.0 + value * 500.0  # -1..1 -> 1000..2000

    def smooth(self, new_val, prev_val, alpha):
        return alpha * new_val + (1.0 - alpha) * prev_val

    def joy_callback(self, msg):
        current_time = time.time()

        if current_time - self.last_publish_time < self.dt_max:
            return

        try:
            # Map PS5 axes (standard joy mapping)
            # axes[0]: left_x (yaw)
            # axes[1]: left_y (throttle, inverted)
            # axes[2]: right_x (roll)
            # axes[3]: right_y (pitch, inverted for forward)
            if len(msg.axes) < 4:
                self.get_logger().warn('Joy message has insufficient axes')
                return

            raw_roll = self.apply_deadzone(msg.axes[2], self.deadzone)  # right_x
            raw_pitch = self.apply_deadzone(-msg.axes[3], self.deadzone)  # right_y inverted for forward pitch
            raw_yaw = self.apply_deadzone(msg.axes[0], self.deadzone)  # left_x
            raw_throttle = self.apply_deadzone(msg.axes[1], self.deadzone)  # left_y

            # Convert to PWM
            pwm_roll = self.joy_to_pwm(raw_roll)
            pwm_pitch = self.joy_to_pwm(raw_pitch)
            pwm_yaw = self.joy_to_pwm(raw_yaw)
            pwm_throttle = self.joy_to_pwm(raw_throttle, is_throttle=True)

            # Smooth
            self.smoothed_axes['roll'] = self.smooth(pwm_roll, self.smoothed_axes['roll'], self.smoothing_alpha)
            self.smoothed_axes['pitch'] = self.smooth(pwm_pitch, self.smoothed_axes['pitch'], self.smoothing_alpha)
            self.smoothed_axes['yaw'] = self.smooth(pwm_yaw, self.smoothed_axes['yaw'], self.smoothing_alpha)
            self.smoothed_axes['throttle'] = self.smooth(pwm_throttle, self.smoothed_axes['throttle'], self.smoothing_alpha)

            # Create Joy message for RC (axes order: roll, pitch, yaw, throttle)
            rc_msg = Joy()
            rc_msg.header.stamp = self.get_clock().now().to_msg()
            rc_msg.header.frame_id = 'rc_input'
            rc_msg.axes = [
                self.smoothed_axes['roll'],
                self.smoothed_axes['pitch'],
                self.smoothed_axes['yaw'],
                self.smoothed_axes['throttle']
            ]
            rc_msg.buttons = msg.buttons  # Pass buttons through

            self.manual_rc_pub.publish(rc_msg)
            self.last_publish_time = current_time

            self.get_logger().debug('Published RC: roll={:.0f}, pitch={:.0f}, yaw={:.0f}, throttle={:.0f}'.format(
                rc_msg.axes[0], rc_msg.axes[1], rc_msg.axes[2], rc_msg.axes[3]))

        except Exception as e:
            self.get_logger().error(f'Error processing joy: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PS5ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
