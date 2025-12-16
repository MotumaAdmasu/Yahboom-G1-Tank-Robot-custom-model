#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

class TankController(Node):
    def __init__(self):
        super().__init__('tank_controller')
        
        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Current wheel positions
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        
        # Timer to continuously publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        
        self.get_logger().info('Tank Controller Ready!')
        self.get_logger().info('Use: ros2 run teleop_twist_keyboard teleop_twist_keyboard')
        self.get_logger().info('W - Forward, S - Backward, A - Turn Left, D - Turn Right')

    def cmd_vel_callback(self, msg):
        # Convert twist commands to wheel velocities (tank drive)
        # linear.x: forward/backward speed (m/s)
        # angular.z: rotation speed (rad/s)
        
        # Tank drive equations:
        # left_velocity = linear - (angular * wheel_separation / 2)
        # right_velocity = linear + (angular * wheel_separation / 2)
        
        wheel_separation = 0.16  # Distance between tracks in meters
        wheel_radius = 0.04     # Wheel radius in meters
        
        # Convert linear velocity to angular velocity (rad/s)
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Calculate wheel angular velocities
        left_angular_vel = (linear_vel - (angular_vel * wheel_separation / 2.0)) / wheel_radius
        right_angular_vel = (linear_vel + (angular_vel * wheel_separation / 2.0)) / wheel_radius
        
        # Scale for better visualization and update positions
        time_step = 0.1  # seconds
        scale = 3.0      # Make movement more visible
        
        self.left_wheel_pos += left_angular_vel * time_step * scale
        self.right_wheel_pos += right_angular_vel * time_step * scale
        
        # Log the commands (limit logging to avoid spam)
        self.get_logger().info(f'Forward: {msg.linear.x:.2f} m/s, Turn: {msg.angular.z:.2f} rad/s')
        self.get_logger().info(f'Wheels: L={left_angular_vel:.2f} rad/s, R={right_angular_vel:.2f} rad/s')

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'chassis'
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = []
        joint_state.effort = []
        
        self.joint_pub.publish(joint_state)

def main():
    rclpy.init()
    node = TankController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
