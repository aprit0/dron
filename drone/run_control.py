#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

# Import node classes
from control.ps5_controller_node import PS5ControllerNode
from control.safety_mixer_node import SafetyMixerNode


def main(args=None):
    rclpy.init(args=args)

    # Create nodes

    ps5_node = PS5ControllerNode()
    mixer_node = SafetyMixerNode()

    # Use multithreaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(ps5_node)
    executor.add_node(mixer_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        ps5_node.destroy_node()
        mixer_node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
