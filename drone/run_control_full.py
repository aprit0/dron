#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

# Import all control node classes
from control.ps5_controller_node import PS5ControllerNode
from control.safety_mixer_node import SafetyMixerNode
from control.mission_planner_node import MissionPlannerNode
from web.dashboard_node import WebDashboardNode

def main(args=None):
    rclpy.init(args=args)

    # Create all nodes for full stack
    ps5_node = PS5ControllerNode()
    mixer_node = SafetyMixerNode()
    mission_node = MissionPlannerNode()
    dashboard_node = WebDashboardNode()

    # Multi-threaded executor for independent debugging
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ps5_node)
    executor.add_node(mixer_node)
    executor.add_node(mission_node)
    executor.add_node(dashboard_node)

    print("Full UAV Control Stack Launched:")
    print("- PS5 Manual Control (/joy -> /manual/rc_out)")
    print("- Safety Mixer (/manual & /auto -> /multiwii/rc_in)")
    print("- Mission Planner (services/topics -> /auto/rc_out)")
    print("- Web Dashboard (http://localhost:8000)")
    print("Run alongside: python3 drone/run_sensors.py")
    print("PS5: ros2 run joy joy_node")
    print("Debug: Ctrl+C to stop gracefully")

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        ps5_node.destroy_node()
        mixer_node.destroy_node()
        mission_node.destroy_node()
        dashboard_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
