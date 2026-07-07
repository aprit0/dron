#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor

# Import node classes
from sensors.msp_node_new import MultiWiiRosNode
from sensors.of_node import CV0850
from sensors.gps_node import GPSNode


def main(args=None):
    rclpy.init(args=args)

    # Create nodes

    msp_node = MultiWiiRosNode()
    of_node = CV0850()
    gps_node = GPSNode()

    # Use multithreaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(msp_node)
    executor.add_node(of_node)
    executor.add_node(gps_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        msp_node.destroy_node()
        of_node.destroy_node()
        gps_node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
