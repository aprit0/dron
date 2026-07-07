#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    
    # Create a temporary node to inspect the system
    node = Node('node_inspector')
    
    # Get all active nodes
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    
    print(f"{'Node':<30} {'Direction':<12} {'Topic':<30} {'Message Type'}")
    print("="*90)
    
    for node_name, node_ns in node_names_and_namespaces:
        full_node_name = f"{node_ns}/{node_name}" if node_ns != "/" else node_name
        try:
            # Get node info
            node_info = node.get_node_names_and_namespaces()
            
            # List publishers
            pub_info = node.get_publisher_names_and_types_by_node(node_name, node_ns)
            for topic, types in pub_info:
                for t in types:
                    print(f"{full_node_name:<30} {'Publisher':<12} {topic:<30} {t}")
            
            # List subscribers
            sub_info = node.get_subscriber_names_and_types_by_node(node_name, node_ns)
            for topic, types in sub_info:
                for t in types:
                    print(f"{full_node_name:<30} {'Subscriber':<12} {topic:<30} {t}")
        except Exception as e:
            print(f"{full_node_name:<30} {'Error':<12} {str(e)}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
