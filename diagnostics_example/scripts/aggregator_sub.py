#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class DiagnosticsTreeNode:
    def __init__(self, status):
        self.status = status
        self.children = []

class DiagnosticsSubscriber(Node):
    def __init__(self):
        super().__init__('diagnostics_subscriber')
        self.subscription = self.create_subscription(
            DiagnosticArray,
            'diagnostics_agg',
            self.diagnostics_callback,
            10
        )
        self.root = None

    def diagnostics_callback(self, msg):
        self.get_logger().info("Received diagnostics message")
        self.root = DiagnosticsTreeNode(None)
        for status in msg.status:
            self.build_tree(status, self.root)
            self.print_tree(self.root)

    def build_tree(self, status, parent_node):
        new_node = DiagnosticsTreeNode(status)
        parent_node.children.append(new_node)

        for value in status.values:
            sub_status = DiagnosticStatus(
                level=status.level,  # Use the parent status level
                name=f"{status.name}/{value.key}",
                message=value.value,
                hardware_id=status.hardware_id,
                values=[]  # Assuming there are no nested values for KeyValue
            )
            self.build_tree(sub_status, new_node)

    def print_tree(self, node, depth=0):
        if node.status is not None:
            indentation = '  ' * depth
            self.get_logger().info(f"{indentation}Level: {node.status.level}")
            self.get_logger().info(f"{indentation}Name: {node.status.name}")
            self.get_logger().info(f"{indentation}Message: {node.status.message}")
            self.get_logger().info(f"{indentation}Hardware ID: {node.status.hardware_id}")
            for value in node.status.values:
                self.get_logger().info(f"{indentation}  {value.key}: {value.value}")

        for child_node in node.children:
            self.print_tree(child_node, depth + 1)

def main(args=None):
    rclpy.init(args=args)
    diagnostics_subscriber = DiagnosticsSubscriber()
    rclpy.spin(diagnostics_subscriber)
    diagnostics_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
