#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

# Import node classes
from sensors.msp_node import MultiWiiRosNode
from sensors.arduino_node import ArduinoSerialNode


def main(args=None):
    rclpy.init(args=args)

    # Create nodes

    msp_node = MultiWiiRosNode()
    arduino_node = ArduinoSerialNode()

    # Use multithreaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(msp_node)
    executor.add_node(arduino_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        msp_node.destroy_node()
        arduino_node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
