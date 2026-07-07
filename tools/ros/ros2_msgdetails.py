#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.task import Future
import importlib
import time
from tabulate import tabulate


class IntrospectionNode(Node):
    def __init__(self):
        super().__init__('ros2_introspection_table')

    def get_message_class(self, type_string):
        """
        Convert 'std_msgs/msg/String' -> Python message class
        """
        try:
            pkg, _, msg = type_string.split('/')
            module = importlib.import_module(f"{pkg}.msg")
            return getattr(module, msg)
        except Exception:
            return None

    def get_sample_message(self, topic_name, type_string, timeout=2.0):
        """
        Subscribe temporarily to get one message sample
        """
        msg_class = self.get_message_class(type_string)
        if msg_class is None:
            return "Unknown type"

        future = Future()

        def callback(msg):
            if not future.done():
                future.set_result(str(msg)[:200])  # truncate long messages

        sub = self.create_subscription(
            msg_class,
            topic_name,
            callback,
            10
        )

        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                break

        self.destroy_subscription(sub)

        if future.done():
            return future.result()
        return "No message received"


def main():
    rclpy.init()
    node = IntrospectionNode()

    rows = []

    topic_types = dict(node.get_topic_names_and_types())

    for node_name, node_ns in node.get_node_names_and_namespaces():
        publishers = node.get_publisher_names_and_types_by_node(node_name, node_ns)
        subscribers = node.get_subscriber_names_and_types_by_node(node_name, node_ns)

        for topic, types in publishers:
            msg_type = types[0]
            sample = node.get_sample_message(topic, msg_type)
            rows.append([node_name, "Publisher", topic, msg_type, sample])

        for topic, types in subscribers:
            msg_type = types[0]
            rows.append([node_name, "Subscriber", topic, msg_type, "-"])

    print(tabulate(
        rows,
        headers=["Node", "Direction", "Topic", "Message Type", "Sample Message"],
        tablefmt="grid"
    ))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()