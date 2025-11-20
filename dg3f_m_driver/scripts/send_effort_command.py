#!/usr/bin/env python3
# Copyright 2025 TESOLLO
#
# Example script to send effort commands to the dg3f gripper

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math


class EffortCommandPublisher(Node):
    def __init__(self):
        super().__init__('effort_command_publisher')
        
        # Publisher to send effort commands
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )
        
        # Timer to publish commands at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Counter for generating sine wave pattern
        self.counter = 0.0
        
        self.get_logger().info('Effort Command Publisher started')
        self.get_logger().info('Publishing to /effort_controller/commands')

    def timer_callback(self):
        msg = Float64MultiArray()
        
        # Example 1: Send constant effort to all joints
        # msg.data = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        
        # Example 2: Send sine wave effort (smooth oscillation)
        effort = 1.0 * math.sin(self.counter)
        msg.data = [effort] * 12  # 12 joints
        
        # Example 3: Send different efforts to different fingers
        # finger1 = [0.5, 0.5, 0.5, 0.5]
        # finger2 = [1.0, 1.0, 1.0, 1.0]
        # finger3 = [0.2, 0.2, 0.2, 0.2]
        # msg.data = finger1 + finger2 + finger3
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing effort: {effort:.2f}')
        
        # Increment counter for sine wave
        self.counter += 0.1


def main(args=None):
    rclpy.init(args=args)
    
    effort_publisher = EffortCommandPublisher()
    
    try:
        rclpy.spin(effort_publisher)
    except KeyboardInterrupt:
        pass
    
    effort_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
