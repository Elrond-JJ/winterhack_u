#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script for hybrid alignment system.
Compares base-only vs hybrid mode performance.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from winterhack_interfaces.action import Locate
import time


class AlignmentTester(Node):
    def __init__(self):
        super().__init__('alignment_tester')
        self.locate_client = ActionClient(self, Locate, '/locate')
        
    def test_alignment(self, mode_name: str):
        """Test alignment and measure performance."""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Testing alignment mode: {mode_name}")
        self.get_logger().info(f"{'='*60}")
        
        if not self.locate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Locate action server not available")
            return None
        
        goal = Locate.Goal()
        
        start_time = time.time()
        send_goal_future = self.locate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return None
        
        self.get_logger().info("Goal accepted, waiting for result...")
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        elapsed_time = time.time() - start_time
        result = result_future.result()
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Results for {mode_name}:")
        self.get_logger().info(f"  Success: {result.result.success}")
        self.get_logger().info(f"  Message: {result.result.message}")
        self.get_logger().info(f"  Time: {elapsed_time:.2f}s")
        self.get_logger().info(f"{'='*60}\n")
        
        return {
            'mode': mode_name,
            'success': result.result.success,
            'message': result.result.message,
            'time': elapsed_time,
        }


def main():
    rclpy.init()
    tester = AlignmentTester()
    
    print("\n" + "="*60)
    print("HYBRID ALIGNMENT TEST")
    print("="*60)
    print("\nThis test will evaluate the alignment system.")
    print("\nMake sure:")
    print("  1. locate_server is running")
    print("  2. A colored object is visible")
    print("  3. Detection node is publishing")
    print("\n" + "="*60 + "\n")
    
    input("Press ENTER to start test...")
    
    # Test current configuration
    result = tester.test_alignment("Current Configuration")
    
    if result and result['success']:
        print("\n" + "="*60)
        print("✅ ALIGNMENT TEST PASSED")
        print("="*60)
        print(f"\n  Mode: {result['mode']}")
        print(f"  Time: {result['time']:.2f}s")
        print(f"  Message: {result['message']}")
        print("\n" + "="*60 + "\n")
    else:
        print("\n" + "="*60)
        print("❌ ALIGNMENT TEST FAILED")
        print("="*60)
        if result:
            print(f"\n  Error: {result['message']}")
        print("\n" + "="*60 + "\n")
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
