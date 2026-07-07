#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy, NavSatFix, Imu
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
from rclpy.duration import Duration
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import json
import math
import time
import threading
from typing import List, Dict, Tuple
from dataclasses import dataclass
from enum import Enum

@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float = 0.0
    yaw: float = 0.0  # degrees

class MissionState(Enum):
    IDLE = 0
    TAKEOFF = 1
    WAYPOINT_FOLLOW = 2
    LAND = 3
    RTB = 4
    PAUSED = 5
    EMERGENCY = 6

class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__('mission_planner_node')

        # Publishers
        self.auto_rc_pub = self.create_publisher(Joy, '/auto/rc_out', 10)

        # Subscribers for telemetry
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/multiwii/imu', self.imu_callback, 10)
        self.alt_sub = self.create_subscription(Float32, '/multiwii/altitude', self.alt_callback, 10)
        self.mode_sub = self.create_subscription(String, '/control_mode', self.mode_callback, 10)

        # Subscribers for mission commands
        self.waypoints_sub = self.create_subscription(String, '/mission/waypoints_json', self.waypoints_callback, 10)
        self.target_alt_sub = self.create_subscription(Float32, '/mission/target_alt', self.target_alt_callback, 10)

        # Services
        self.start_srv = self.create_service(Trigger, '/mission/start', self.start_callback)
        self.pause_srv = self.create_service(Trigger, '/mission/pause', self.pause_callback)
        self.resume_srv = self.create_service(Trigger, '/mission/resume', self.resume_callback)
        self.land_srv = self.create_service(Trigger, '/mission/land', self.land_callback)
        self.rtb_srv = self.create_service(Trigger, '/mission/rtb', self.rtb_callback)
        self.takeoff_srv = self.create_service(Trigger, '/mission/takeoff', self.takeoff_callback)

        # State
        self.state = MissionState.IDLE
        self.waypoints: List[Waypoint] = []
        self.current_wp_idx = 0
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.current_yaw = 0.0  # rad
        self.current_n = 0.0
        self.current_e = 0.0
        self.gps_hdop = 99.0
        self.control_mode = 'manual'
        self.target_alt = 0.0
        self.wp_radius = 3.0  # m
        self.max_speed = 5.0  # m/s
        self.pid_gains = {
            'pos_p': 0.5, 'pos_i': 0.0, 'pos_d': 0.1,
            'alt_p': 1.0, 'alt_i': 0.0, 'alt_d': 0.2
        }

        # PID instances (simple)
        self.pos_pid_n = self.PID(self.pid_gains['pos_p'], self.pid_gains['pos_i'], self.pid_gains['pos_d'])
        self.pos_pid_e = self.PID(self.pid_gains['pos_p'], self.pid_gains['pos_i'], self.pid_gains['pos_d'])
        self.alt_pid = self.PID(self.pid_gains['alt_p'], self.pid_gains['alt_i'], self.pid_gains['alt_d'])

        # Timers
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz

        # Mission state publisher
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        self.get_logger().info('Mission Planner Node initialized')

    class PID:
        def __init__(self, kp, ki, kd):
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.prev_error = 0.0
            self.integral = 0.0

        def compute(self, setpoint, measurement, dt):
            error = setpoint - measurement
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.prev_error = error
            return output

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
        self.gps_hdop = msg.position_covariance[0]**0.5 if msg.position_covariance_type != 0 else 99.0
        self.current_n, self.current_e = self.gps_to_local(self.current_lat, self.current_lon)

    def imu_callback(self, msg):
        # Extract yaw from quaternion
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.current_yaw = math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]), 1.0 - 2.0 * (q[1]**2 + q[2]**2))

    def alt_callback(self, msg):
        self.current_alt = msg.data

    def mode_callback(self, msg):
        self.control_mode = msg.data

    def waypoints_callback(self, msg):
        try:
            wp_list = json.loads(msg.data)
            self.waypoints = [Waypoint(**wp) for wp in wp_list]
            self.current_wp_idx = 0
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Waypoint load error: {e}')

    def target_alt_callback(self, msg):
        self.target_alt = msg.data

    def load_mission_callback(self, request, response):
        pass  # deprecated
        response.success = True
        response.message = 'Use /mission/waypoints_json topic'
        return response

    def start_callback(self, request, response):
        response.success = True
        response.message = 'Mission started'
        if self.control_mode != 'auto':
            response.success = False
            response.message = 'Not in auto mode'
            return response
        self.state = MissionState.WAYPOINT_FOLLOW if self.waypoints else MissionState.IDLE
        self.get_logger().info('Mission started')
        return response

    def pause_callback(self, request, response):
        self.state = MissionState.PAUSED
        self.get_logger().info('Mission paused')
        response.success = True
        response.message = 'Paused'
        return response

    def resume_callback(self, request, response):
        if self.state == MissionState.PAUSED:
            self.state = MissionState.WAYPOINT_FOLLOW
        response.success = True
        response.message = 'Resumed'
        return response

    def land_callback(self, request, response):
        self.state = MissionState.LAND
        response.success = True
        response.message = 'Landing'
        return response

    def rtb_callback(self, request, response):
        self.state = MissionState.RTB
        response.success = True
        response.message = 'Returning to base'
        return response

    def takeoff_callback(self, request, response):
        self.state = MissionState.TAKEOFF
        response.success = True
        response.message = 'Takeoff initiated'
        return response

    def gps_to_local(self, lat, lon) -> Tuple[float, float]:
        # Simple flat earth approx
        dlat = math.radians(lat - self.home_lat)
        dlon = math.radians(lon - self.home_lon) * math.cos(math.radians(self.home_lat))
        north = dlat * 6371000
        east = dlon * 6371000
        return north, east

    def local_to_rc(self, north_cmd, east_cmd, alt_cmd, yaw_cmd):
        # PID for position to velocity cmd (simple proportional)
        vel_n = self.pos_pid_n.compute(north_cmd, 0.0, 0.02)  # target north error -> vel_n
        vel_e = self.pos_pid_e.compute(east_cmd, 0.0, 0.02)
        throttle_cmd = self.alt_pid.compute(alt_cmd, self.current_alt, 0.02)

        # Vel to attitude rates (simple)
        pitch_rate = vel_n * 0.5  # rad/s
        roll_rate = -vel_e * 0.5
        yaw_rate = yaw_cmd * 0.1

        # Rates to PWM (scale to +/-500 around 1500)
        roll_pwm = 1500 + roll_rate * 200
        pitch_pwm = 1500 + pitch_rate * 200
        yaw_pwm = 1500 + yaw_rate * 200
        throttle_pwm = 1500 + throttle_cmd * 2  # scale

        return [roll_pwm, pitch_pwm, yaw_pwm, throttle_pwm]

    def control_loop(self):
        if self.control_mode != 'auto' or self.state == MissionState.IDLE or self.state == MissionState.PAUSED or self.state == MissionState.EMERGENCY:
            self.publish_neutral()
            return

        # Publish state
        state_msg = String()
        state_msg.data = self.state.name.lower()
        self.state_pub.publish(state_msg)

        dt = 0.02

        # Set home if not set
        if self.home_lat == 0.0:
            self.home_lat, self.home_lon, self.home_alt = self.current_lat, self.current_lon, self.current_alt

        # State machine
        if self.state == MissionState.TAKEOFF:
            if self.current_alt >= self.target_alt - 0.5:
                self.state = MissionState.WAYPOINT_FOLLOW
            rc = self.local_to_rc(0, 0, self.target_alt, 0)
        elif self.state == MissionState.WAYPOINT_FOLLOW:
            if self.waypoints:
                wp = self.waypoints[self.current_wp_idx]
                target_n, target_e = self.gps_to_local(wp.lat, wp.lon)
                dist = math.sqrt((target_n - self.current_n)**2 + (target_e - self.current_e)**2)
                if dist < self.wp_radius:
                    self.current_wp_idx += 1
                    if self.current_wp_idx >= len(self.waypoints):
                        self.state = MissionState.IDLE
                        self.publish_neutral()
                        return
                rc = self.local_to_rc(target_n - self.current_n, target_e - self.current_e, wp.alt, wp.yaw)
            else:
                rc = self.local_to_rc(0, 0, self.current_alt, 0)  # hold
        elif self.state == MissionState.LAND:
            if self.current_alt < 0.5:
                self.state = MissionState.IDLE
            rc = self.local_to_rc(0, 0, 0.0, 0)
        elif self.state == MissionState.RTB:
            target_n, target_e = 0.0, 0.0  # relative to home
            dist = math.sqrt((target_n - self.current_n)**2 + (target_e - self.current_e)**2)
            if dist < self.wp_radius:
                self.state = MissionState.LAND
            rc = self.local_to_rc(target_n - self.current_n, target_e - self.current_e, self.home_alt + 2.0, 0)

        # Sanity check
        if self.gps_hdop > 2.0:
            self.state = MissionState.EMERGENCY
            self.get_logger().warn('Poor GPS HDOP, emergency hold')

        # Clamp and publish
        rc = [max(1000, min(2000, v)) for v in rc]
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = rc
        self.auto_rc_pub.publish(msg)

    def publish_neutral(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [1500.0, 1500.0, 1500.0, 1000.0]
        self.auto_rc_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
