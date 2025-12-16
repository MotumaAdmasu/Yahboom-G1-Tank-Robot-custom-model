#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

class TankController(Node):
    def __init__(self):
        super().__init__('tank_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        
        self.get_logger().info('Tank Controller Ready - Use ROS2 teleop_twist_keyboard to control')

    def listener_callback(self, msg):
        # Convert twist to wheel movements (tank drive)
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Tank drive: each track moves independently
        left_speed = linear - angular
        right_speed = linear + angular
        
        # Update wheel positions
        self.left_wheel_pos += left_speed
        self.right_wheel_pos += right_speed
        
        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        
        self.publisher.publish(joint_state)
        
        self.get_logger().info(f'Moving: left={left_speed:.2f}, right={right_speed:.2f}')

def main():
    rclpy.init()
    node = TankController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
