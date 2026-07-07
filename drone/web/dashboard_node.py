#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import json
import time
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from sensor_msgs.msg import Joy, NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Int32
from rosidl_runtime_py import import_message
import math

class WebDashboardNode(Node):
    def __init__(self):
        super().__init__('web_dashboard_node')

        # FastAPI app
        self.app = FastAPI()
        self.app.mount("/static", StaticFiles(directory="web/static"), name="static")

        # WebSocket clients
        self.websocket_clients = set()

        # Telemetry storage (latest)
        self.telemetry = {
            'gps': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0, 'hdop': 99.0, 'sats': 0},
            'imu': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'of_odometry': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'voltage': 0.0,
            'rc_in': {'roll': 1500, 'pitch': 1500, 'yaw': 1500, 'throttle': 1000},
            'rc_out': {'roll': 1500, 'pitch': 1500, 'yaw': 1500, 'throttle': 1000},
            'rc': {'in': {'roll': 1500, 'pitch': 1500, 'yaw': 1500, 'throttle': 1000}, 'out': {'roll': 1500, 'pitch': 1500, 'yaw': 1500, 'throttle': 1000}},
            'state': 'disarm',
            'topics': []
        }

        # ROS subscriptions
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.gps_sats = self.create_subscription(Int32, '/gps/satellites', self.gps_sats_cb, 10)
        self.of_odometry_sub = self.create_subscription(Odometry, '/optical_flow/odom', self.of_odom_cb, 10)
        self.rc_in_sub = self.create_subscription(Joy, '/multiwii/rc_in', self.rc_in_cb, 10)
        self.rc_out_sub = self.create_subscription(Joy, '/multiwii/rc_out', self.rc_out_cb, 10)
        self.voltage_sub = self.create_subscription(Float32, '/multiwii/voltage', self.voltage_cb, 10)

        # Flight state subscription
        self.state_sub = self.create_subscription(String, '/mission/state', self.mission_cb, 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.manual_rc_pub = self.create_publisher(Joy, '/multiwii/rc_in', 10)

        self.manual_publish_dt = 0.05
        self.last_manual_publish_time = 0.0

        self.topic_activity = {}
        self.topic_subscriptions = {}
        self.ignored_topics = {
            '/parameter_events', '/rosout', '/rosout_agg', '/clock', '/tf', '/tf_static',
            '/diagnostics', '/diagnostics_agg', '/events', '/performance_metrics', '/connected_clients'
        }
        
        # Telemetry broadcast timer
        self.timer = self.create_timer(0.1, self.broadcast_telemetry)  # 10Hz
        self.topic_timer = self.create_timer(1.0, self.refresh_topic_registry)

        # Setup routes
        self.setup_routes()

        # Start FastAPI server in thread
        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info('Web Dashboard Node started. Open http://localhost:8000')

    def _should_ignore_topic(self, topic_name):
        return topic_name in self.ignored_topics or topic_name.startswith('/_')

    def _mark_topic_seen(self, topic_name, msg_type, last_message=None):
        if self._should_ignore_topic(topic_name):
            return
        if topic_name not in self.topic_activity:
            self.topic_activity[topic_name] = {
                'name': topic_name,
                'type': msg_type,
                'last_seen': time.time(),
                'last_message': last_message if last_message is not None else ''
            }
        else:
            self.topic_activity[topic_name]['type'] = msg_type
            self.topic_activity[topic_name]['last_seen'] = time.time()
            if last_message is not None:
                self.topic_activity[topic_name]['last_message'] = last_message

    def refresh_topic_registry(self):
        try:
            topics_and_types = self.get_topic_names_and_types()
        except Exception as exc:
            self.get_logger().debug(f'Topic discovery failed: {exc}')
            return

        current_topics = set()
        for topic_name, type_names in topics_and_types:
            if self._should_ignore_topic(topic_name):
                continue

            publisher_count = self.count_publishers(topic_name)
            if publisher_count == 0:
                continue

            current_topics.add(topic_name)
            msg_type = type_names[0] if type_names else 'unknown'
            self._mark_topic_seen(topic_name, msg_type)

            if topic_name not in self.topic_subscriptions:
                try:
                    msg_class = import_message(msg_type)
                    self.topic_subscriptions[topic_name] = self.create_subscription(
                        msg_class,
                        topic_name,
                        lambda msg, topic_name=topic_name, msg_type=msg_type: self._mark_topic_seen(topic_name, msg_type, str(msg)),
                        10,
                    )
                except Exception as exc:
                    self.get_logger().debug(f'Unable to subscribe to {topic_name}: {exc}')

        for topic_name in list(self.topic_activity):
            if topic_name not in current_topics:
                self.topic_activity.pop(topic_name, None)
                self.topic_subscriptions.pop(topic_name, None)

    def get_topic_statuses(self):
        now = time.time()
        statuses = []
        for topic_name, info in sorted(self.topic_activity.items()):
            age = max(0.0, now - info['last_seen'])
            statuses.append({
                'name': topic_name,
                'type': info['type'],
                'age': round(age, 2),
                'stale': age > 2.0,
                'last_message': info.get('last_message', ''),
            })
        return statuses

    def gps_cb(self, msg):
        self.telemetry['gps'] = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'hdop': math.sqrt(msg.position_covariance[0]) if msg.position_covariance_type != 0 else 99.0,
        }
        self._mark_topic_seen('/gps/fix', 'sensor_msgs/msg/NavSatFix', str(msg))

    def gps_sats_cb(self, msg):
        self.telemetry['gps']['sats'] = msg.data
        self._mark_topic_seen('/gps/satellites', 'std_msgs/msg/Int32', str(msg))
        
    def of_odom_cb(self, msg):
        self.telemetry['of_odometry'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        roll = math.atan2(2*(q[3]*q[0] + q[1]*q[2]), 1 - 2*(q[0]**2 + q[1]**2))
        pitch = math.asin(2*(q[3]*q[1] - q[2]*q[0]))
        yaw = math.atan2(2*(q[3]*q[2] + q[0]*q[1]), 1 - 2*(q[1]**2 + q[2]**2))
        self.telemetry['imu'] = {'roll': math.degrees(roll), 'pitch': math.degrees(pitch), 'yaw': math.degrees(yaw)}
        self._mark_topic_seen('/optical_flow/odom', 'nav_msgs/msg/Odometry', str(msg))

    def _update_rc_state(self, target_key, topic_name, msg):
        if len(msg.axes) >= 4:
            self.telemetry[target_key] = {
                'roll': msg.axes[0],
                'pitch': msg.axes[1],
                'yaw': msg.axes[2],
                'throttle': msg.axes[3]
            }
            self.telemetry['rc'] = {
                'in': self.telemetry.get('rc_in', {'roll': 1500, 'pitch': 1500, 'yaw': 1500, 'throttle': 1000}),
                'out': self.telemetry.get('rc_out', {'roll': 1500, 'pitch': 1500, 'yaw': 1500, 'throttle': 1000})
            }
        self._mark_topic_seen(topic_name, 'sensor_msgs/msg/Joy', str(msg))

    def rc_in_cb(self, msg):
        self._update_rc_state('rc_in', '/multiwii/rc_in', msg)

    def rc_out_cb(self, msg):
        self._update_rc_state('rc_out', '/multiwii/rc_out', msg)

    def voltage_cb(self, msg):
        self.telemetry['voltage'] = float(msg.data)
        self._mark_topic_seen('/multiwii/voltage', 'std_msgs/msg/Float32', str(msg))

    def mission_cb(self, msg):
        self.telemetry['state'] = msg.data
        self._mark_topic_seen('/mission/state', 'std_msgs/msg/String', str(msg))

    async def broadcast_telemetry(self):
        self.telemetry['topics'] = self.get_topic_statuses()
        data = json.dumps(self.telemetry)
        for ws in list(self.websocket_clients):
            try:
                await ws.send_text(data)
            except:
                self.websocket_clients.discard(ws)

    def get_index(self):
        try:
            with open('web/static/index.html', 'r') as f:
                html = f.read()
        except FileNotFoundError:
            html = "<h1>UI not found. Create web/static/index.html</h1>"
        return HTMLResponse(content=html)

    async def websocket_endpoint(self, websocket: WebSocket):
        await websocket.accept()
        self.websocket_clients.add(websocket)
        try:
            while True:
                data = await websocket.receive_text()
                await self.handle_command(websocket, json.loads(data))
        except WebSocketDisconnect:
            self.websocket_clients.discard(websocket)
        except Exception as e:
            self.get_logger().error(f'WS error: {e}')

    def publish_manual_rc(self, cmd):
        current_time = time.time()
        if current_time - self.last_manual_publish_time < self.manual_publish_dt:
            return

        roll = float(cmd.get('roll', 1500.0))
        pitch = float(cmd.get('pitch', 1500.0))
        yaw = float(cmd.get('yaw', 1500.0))
        throttle = float(cmd.get('throttle', 1000.0))

        values = [
            max(1000.0, min(2000.0, roll)),
            max(1000.0, min(2000.0, pitch)),
            max(1000.0, min(2000.0, yaw)),
            max(1000.0, min(2000.0, throttle)),
        ]
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'web_manual'
        msg.axes = values
        msg.buttons = []
        self.manual_rc_pub.publish(msg)
        self.last_manual_publish_time = current_time

    async def handle_command(self, ws, cmd):
        try:
            cmd_type = cmd['type']
            if cmd_type == 'set_state':
                msg = String()
                msg.data = cmd['state']
                self.state_pub.publish(msg)
            elif cmd_type == 'manual_update':
                self.publish_manual_rc(cmd)
            else:
                await ws.send_text(json.dumps({'success': False, 'error': f'Unknown command: {cmd_type}'}))
            self.get_logger().info(f'Web cmd: {cmd_type}')
        except Exception as e:
            self.get_logger().error(f'Web cmd error: {e}')
            await ws.send_text(json.dumps({'success': False, 'error': str(e)}))

    def setup_routes(self):
        self.app.get("/")(self.get_index)
        self.app.websocket("/ws")(self.websocket_endpoint)

    def run_server(self):
        import uvicorn
        uvicorn.run(self.app, host="0.0.0.0", port=8000)

def main(args=None):
    rclpy.init(args=args)
    node = WebDashboardNode()
    executor = MultiThreadedExecutor()
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
