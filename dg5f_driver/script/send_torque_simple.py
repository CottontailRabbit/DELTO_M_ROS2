#!/usr/bin/env python3
# Copyright 2025 TESOLLO
#
# Oscillate last joint back and forth every 1 second

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class JointOscillator(Node):
    def __init__(self):
        super().__init__('joint12_oscillator')
        
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/dg5f_left_torque_controller/commands',
            10
        )
        
        # Timer to toggle every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Toggle state
        self.toggle = True
        self.get_logger().info('Toggling last joint every 1 second')

    def timer_callback(self):
        msg = Float64MultiArray()
        

        if self.toggle:
            msg.data = [0.0] * 19 + [0.3]   # last joint = 0.3
            self.get_logger().info('Last joint: 0.3')
        else:
            msg.data = [0.0] * 19 + [-0.3]  # last joint = -0.3
            self.get_logger().info('Last joint: -0.3')
        
        self.publisher_.publish(msg)
        self.toggle = not self.toggle


def main():
    rclpy.init()
    
    oscillator = JointOscillator()
    
    try:
        rclpy.spin(oscillator)
    except KeyboardInterrupt:
        pass
    
    oscillator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
