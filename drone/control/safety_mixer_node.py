#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Header, Float32
import time
import math

class SafetyMixerNode(Node):
    def __init__(self):
        super().__init__('safety_mixer_node')

        # Publishers
        self.rc_pub = self.create_publisher(Joy, '/multiwii/rc_in', 10)

        # Subscribers
        self.manual_sub = self.create_subscription(Joy, '/manual/rc_out', self.manual_callback, 10)
        self.auto_sub = self.create_subscription(Joy, '/auto/rc_out', self.auto_callback, 10)
        self.mode_sub = self.create_subscription(String, '/control_mode', self.mode_callback, 10)

        # Safety subscribers (basic)
        self.range_sub = self.create_subscription(Float32, '/ultrasonic/range', self.range_callback, 10)  # example

        # E-Stop service
        self.estop_srv = self.create_service(Trigger, '/safety/estop', self.estop_service_callback)

        # State
        self.control_mode = 'manual'  # default
        self.manual_rc = self.neutral_rc()
        self.auto_rc = self.neutral_rc()
        self.estop_active = False
        self.last_range = 10.0
        self.manual_timeout = 0.5  # sec
        self.auto_timeout = 0.5
        self.last_manual_time = 0.0
        self.last_auto_time = 0.0
        self.last_publish_time = 0.0
        self.publish_rate = 50.0
        self.dt_max = 1.0 / self.publish_rate

        # Publish timer
        self.timer = self.create_timer(0.01, self.publish_loop)  # 100Hz check, rate limited

        self.get_logger().info('Safety Mixer Node initialized. Mode: manual, Rate: {}Hz'.format(self.publish_rate))

    def neutral_rc(self):
        return [1500.0, 1500.0, 1500.0, 1000.0]  # roll, pitch, yaw, throttle (armed low)

    def clamp_pwm(self, value):
        return max(1000.0, min(2000.0, value))

    def manual_callback(self, msg):
        self.manual_rc = [self.clamp_pwm(a) for a in msg.axes[:4]]
        self.last_manual_time = time.time()

    def auto_callback(self, msg):
        self.auto_rc = [self.clamp_pwm(a) for a in msg.axes[:4]]
        self.last_auto_time = time.time()

    def mode_callback(self, msg):
        self.control_mode = msg.data

    def estop_service_callback(self, request, response):
        self.estop_active = True
        self.get_logger().warn('E-STOP SERVICE TRIGGERED - LATCHED')
        response.success = True
        response.message = 'E-Stop activated'
        return response

    def range_callback(self, msg):
        self.last_range = msg.data

    def apply_safety(self, rc):
        current_time = time.time()
        safe_rc = rc[:]

        # E-stop override
        if self.estop_active:
            safe_rc = self.neutral_rc()
            self.get_logger().warn('E-STOP ACTIVE: Neutral RC')
            return safe_rc

        # Proximity limit: reduce throttle if close to ground
        if self.last_range < 0.5:
            safe_rc[3] = min(safe_rc[3], 1200.0)  # limit throttle
            self.get_logger().warn(f'Proximity limit: range={self.last_range:.2f}m, throttle capped')

        # Signal loss: timeout -> neutral
        if current_time - self.last_manual_time > self.manual_timeout and self.control_mode == 'manual':
            safe_rc = self.neutral_rc()
            self.get_logger().warn('Manual signal timeout -> neutral')
        elif current_time - self.last_auto_time > self.auto_timeout and self.control_mode == 'auto':
            safe_rc = self.neutral_rc()
            self.get_logger().warn('Auto signal timeout -> neutral')

        return safe_rc

    def publish_loop(self):
        current_time = time.time()
        if current_time - self.last_publish_time < self.dt_max:
            return

        # Select source
        if self.control_mode == 'auto':
            selected_rc = self.auto_rc
        else:
            selected_rc = self.manual_rc

        # Apply safety
        safe_rc = self.apply_safety(selected_rc)

        # Publish
        rc_msg = Joy()
        rc_msg.header.stamp = self.get_clock().now().to_msg()
        rc_msg.header.frame_id = 'safe_rc'
        rc_msg.axes = safe_rc + [0.0] * (len(self.manual_rc) - 4)  # pad if needed
        rc_msg.buttons = []  # TODO: pass through

        self.rc_pub.publish(rc_msg)
        self.last_publish_time = current_time

        self.get_logger().debug(f'Mode: {self.control_mode}, Safe RC: {safe_rc}')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMixerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
