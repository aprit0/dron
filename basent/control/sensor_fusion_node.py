#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import Imu, MagneticField, Range, Joy, NavSatFix
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

import numpy as np
from scipy.spatial.transform import Rotation as R
from filterpy.kalman import ExtendedKalmanFilter
# import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import csv
import os
from datetime import datetime


# Helper functions
def gps_to_xyz(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """Simple flat-earth conversion: meters relative to reference"""
    # Approximate constants
    R_EARTH = 6378137.0  # meters
    dlat = np.deg2rad(lat - ref_lat)
    dlon = np.deg2rad(lon - ref_lon)
    dx = R_EARTH * dlon * np.cos(np.deg2rad(ref_lat))
    dy = R_EARTH * dlat
    dz = alt - ref_alt
    return np.array([dx, dy, dz])

class SensorFusionNode(Node):
    def __init__(self, debug_csv=False):
        super().__init__('sensor_fusion_node')

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.VOLATILE

        # Subscribers
        self.sub_barometer = self.create_subscription(Float32, '/barometer/altitude', self.barometer_cb, qos)
        self.sub_imu = self.create_subscription(Imu, '/multiwii/imu', self.imu_cb, qos)
        self.sub_mag = self.create_subscription(MagneticField, '/multiwii/mag', self.mag_cb, qos)
        self.sub_alt = self.create_subscription(Float32, '/multiwii/altitude', self.altitude_cb, qos)
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, qos)
        self.sub_laser = self.create_subscription(Range, '/laser/range', self.laser_cb, qos)
        self.sub_ultrasonic = self.create_subscription(Range, '/ultrasonic/range', self.ultrasonic_cb, qos)

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, '/odometry', qos)
        self.pub_pose = self.create_publisher(PoseStamped, '/pose', qos)
        self.pub_twist = self.create_publisher(Twist, '/twist', qos)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # EKF initialization
        # State: [x, y, z, vx, vy, vz, qx, qy, qz, qw]
        self.ekf = ExtendedKalmanFilter(dim_x=10, dim_z=10)
        self.ekf.x = np.zeros(10)
        self.ekf.P *= 0.1

        # Process noise
        self.ekf.Q = np.eye(10) * 0.01
        # Measurement noise
        self.ekf.R = np.eye(10) * 0.05


        # Previous time
        self.prev_time = self.get_clock().now()

        # Sensor storage
        self.imu_data = None
        self.mag_data = None
        self.barometer = None
        self.altitude = None
        self.gps_data = None
        self.laser = None
        self.ultrasonic = None
        self.barometer_var = 0.5 ** 2      # variance in meters^2
        self.altitude_var = 0.3 ** 2       # internal altitude sensor
        self.laser_var = 0.05 ** 2         # laser altimeter is very precise <1m
        self.ultrasonic_var = 0.1 ** 2     # ultrasonic sensor
        self.gps_var = 1.0 ** 2            # GPS vertical accuracy
        # Calibration tracking
        self.bar_alt_history = {
            'barometer': [],
            'altimeter': []
        }
        self.calibration_start_time = self.get_clock().now()
        self.bar_alt_offset = 0.0       # offset to apply after calibration
        self.alt_offset = 0.0           # separate offset for altimeter if desired
        self.calibrated = False


        # Reference GPS for local frame
        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
        
        # CSV
        if debug_csv:
            self.init_altitude_logger()
        # Timer for EKF updates
        self.timer = self.create_timer(0.02, self.update_ekf)  # 50 Hz
        

    # Callback functions
    def imu_cb(self, msg: Imu):
        self.imu_data = msg

    def mag_cb(self, msg: MagneticField):
        self.mag_data = msg

    def barometer_cb(self, msg: Float32):
        self.barometer = msg.data

    def altitude_cb(self, msg: Float32):
        # Convert to altitude (simple model)
        self.altitude = msg.data

    def gps_cb(self, msg: NavSatFix):
        self.gps_data = msg
        if self.ref_lat is None:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            self.ref_alt = msg.altitude

    def laser_cb(self, msg: Range):
        self.laser = msg.range

    def ultrasonic_cb(self, msg: Range):
        self.ultrasonic = msg.range

    def init_altitude_logger(self):
        log_dir = os.path.expanduser("~/ros2_ws/src/dron/basent/control/altitude_logs")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.altitude_log_path = os.path.join(
            log_dir, f"altitude_log_{timestamp}.csv"
        )

        self.altitude_log_file = open(self.altitude_log_path, mode="w", newline="")
        self.altitude_csv = csv.writer(self.altitude_log_file)

        # CSV header
        self.altitude_csv.writerow([
            "ros_time_sec",
            "barometer",
            "altitude",
            "laser",
            "ultrasonic",
            "gps",
            "fused_altitude"
        ])    

    def state_transition(self, x, dt):
        """
        EKF prediction step (no IMU propagation).
        State: [px, py, pz, vx, vy, vz, qx, qy, qz, qw]
        """
        new_x = x.copy()

        # Extract state (copies, not views)
        px, py, pz = x[0:3]
        vx, vy, vz = x[3:6]
        qx, qy, qz, qw = x[6:10]

        # Constant-velocity model
        px += vx * dt
        py += vy * dt
        pz += vz * dt

        # Write back
        new_x[0] = px
        new_x[1] = py
        new_x[2] = pz

        # Velocity unchanged
        new_x[3] = vx
        new_x[4] = vy
        new_x[5] = vz

        # Orientation unchanged (IMU updates it)
        new_x[6:10] = [qx, qy, qz, qw]

        return new_x
    
    def measurement_function(self, x):
        """Predict sensor measurements from state"""
        z = np.zeros(10)
        z[0:3] = x[0:3]     # position
        z[3:5] = x[3:5]     # velocities vx, vy
        z[5] = 0.0          # vz is typically measured from alt sensors
        z[6:10] = x[6:10]   # orientation quaternion
        return z

    def update_ekf(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 0.02
        self.prev_time = now

        # --- PREDICT ---
        self.ekf.x = self.state_transition(self.ekf.x, dt)
        F = self.state_transition_jacobian(dt)
        self.ekf.P = F @ self.ekf.P @ F.T + self.ekf.Q
        # self.ekf.P += self.ekf.Q * dt  # simple Euler fallback

        # --- MEASUREMENT VECTOR ---
        z = np.zeros(10)
        valid_measurement = False
        valid_idx = []

        # --- ALTITUDE SENSORS ---
        baro = self.barometer
        alt = self.altitude
        laser_raw = self.laser if self.laser is not None and 0.01 <= self.laser < 5.0 else None
        ultra_raw = self.ultrasonic if self.ultrasonic is not None and 0.1 <= self.ultrasonic <= 5.0 else None
        gps_alt = self.gps_data.altitude if self.gps_data is not None else None
        
        
        # --- Project angled laser/ultrasonic into world z using orientation ---
        q = self.ekf.x[6:10]  # EKF quaternion
        if np.linalg.norm(q) < 1e-6:
            q = np.array([0.0, 0.0, 0.0, 1.0])
        R_world_body = R.from_quat(q).as_matrix()

        def project_to_world_z(measurement, sensor_axis=np.array([0, 0, 1])):
            """
            sensor_axis: vector along which the sensor measures (e.g., laser pointing direction)
            """
            if measurement is None:
                return None
            world_vector = R_world_body @ sensor_axis
            # Project measurement along z-axis
            return measurement * world_vector[2]  # only vertical contribution

        laser = project_to_world_z(laser_raw)  # laser measurement projected to vertical
        ultra = project_to_world_z(ultra_raw)  # ultrasonic measurement projected to vertical
        # --- Collect barometer + altimeter for calibration ---
        if baro is not None:
            self.bar_alt_history['barometer'].append(baro)
        if alt is not None:
            self.bar_alt_history['altimeter'].append(alt)

        elapsed = (now - self.calibration_start_time).nanoseconds * 1e-9
        if (not self.calibrated) and (elapsed >= 10.0) and (laser is not None):
            baro_avg = np.mean(self.bar_alt_history['barometer']) if self.bar_alt_history['barometer'] else 0.0
            alt_avg = np.mean(self.bar_alt_history['altimeter']) if self.bar_alt_history['altimeter'] else 0.0
            self.bar_alt_offset = laser - baro_avg
            self.alt_offset = laser - alt_avg
            self.calibrated = True
            # self.get_logger().info(
            #     f"Barometer calibrated: {self.bar_alt_offset:.3f}, Altimeter calibrated: {self.alt_offset:.3f}"
            # )

        # Apply calibration offsets
        if self.calibrated:
            if baro is not None:
                baro += self.bar_alt_offset
            if alt is not None:
                alt += self.alt_offset

        
        

        # --- Build altitude list for fusion ---
        altitudes = []
        weights = []
        if baro is not None and self.calibrated:
            altitudes.append(baro)
            weights.append(1.0 / self.barometer_var)
        if alt is not None and self.calibrated:
            altitudes.append(alt)
            weights.append(1.0 / self.altitude_var)
        if laser is not None:
            altitudes.append(laser)
            weights.append(1.0 / self.laser_var)
        if ultra is not None:
            altitudes.append(ultra)
            weights.append(1.0 / self.ultrasonic_var)
        if gps_alt is not None:
            altitudes.append(gps_alt)
            weights.append(1.0 / self.gps_var)

        # --- Fuse altitude ---
        if altitudes:
            fused_alt = float(np.average(altitudes, weights=weights))
            z[2] = fused_alt
            valid_measurement = True
            valid_idx.append(2)

        # --- POSITION FROM GPS ---
        if self.gps_data is not None:
            xyz = gps_to_xyz(
                self.gps_data.latitude, self.gps_data.longitude, self.gps_data.altitude,
                self.ref_lat, self.ref_lon, self.ref_alt
            )
            z[0] = xyz[0]
            z[1] = xyz[1]
            valid_measurement = True
            valid_idx.extend([0, 1])

        # --- ORIENTATION FROM IMU ---
        if self.imu_data is not None:
            q_imu = self.imu_data.orientation
            z[6:10] = [q_imu.x, q_imu.y, q_imu.z, q_imu.w]
            valid_measurement = True
            valid_idx.extend([6, 7, 8, 9])

        # --- EKF UPDATE ---
        if valid_measurement:
            H = np.eye(10)[valid_idx]
            R_mat = self.ekf.R[np.ix_(valid_idx, valid_idx)]
            y = z[valid_idx] - self.measurement_function(self.ekf.x)[valid_idx]
            S = H @ self.ekf.P @ H.T + R_mat
            K = self.ekf.P @ H.T @ np.linalg.inv(S)
            self.ekf.x = self.ekf.x + K @ y
            self.ekf.P = (np.eye(10) - K @ H) @ self.ekf.P

        self.publish_messages()



    def publish_messages(self):
        # ====================
        # Publish Odometry
        # ====================
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = self.ekf.x[0]
        odom.pose.pose.position.y = self.ekf.x[1]
        odom.pose.pose.position.z = self.ekf.x[2]

        # Orientation (quaternion)
        odom.pose.pose.orientation.x = self.ekf.x[6]
        odom.pose.pose.orientation.y = self.ekf.x[7]
        odom.pose.pose.orientation.z = self.ekf.x[8]
        odom.pose.pose.orientation.w = self.ekf.x[9]

        # Twist (velocity)
        odom.twist.twist.linear.x = self.ekf.x[3]
        odom.twist.twist.linear.y = self.ekf.x[4]
        odom.twist.twist.linear.z = self.ekf.x[5]

        # Assuming angular velocities are unknown for now
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.pub_odom.publish(odom)

        # ====================
        # Publish PoseStamped
        # ====================
        pose_msg = PoseStamped()
        pose_msg.header = odom.header
        pose_msg.pose = odom.pose.pose
        self.pub_pose.publish(pose_msg)

        # ====================
        # Publish Twist
        # ====================
        twist_msg = Twist()
        twist_msg.linear = odom.twist.twist.linear
        twist_msg.angular = odom.twist.twist.angular
        self.pub_twist.publish(twist_msg)

        # ====================
        # Publish TF
        # ====================
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        # Correct translation assignment using Vector3
        from geometry_msgs.msg import Vector3
        t.transform.translation = Vector3(
            x=odom.pose.pose.position.x,
            y=odom.pose.pose.position.y,
            z=odom.pose.pose.position.z
        )

        # Orientation is directly assignable
        t.transform.rotation = odom.pose.pose.orientation

        # Broadcast TF
        self.tf_broadcaster.sendTransform(t)

def destroy_node(self):
    if hasattr(self, "altitude_log_file"):
        self.altitude_log_file.close()
    super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
