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
        
        # Current wheel positions and velocities
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
        # Timer to continuously publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        
        self.get_logger().info('=== TANK CONTROLLER READY ===')
        self.get_logger().info('Use: ros2 run teleop_twist_keyboard teleop_twist_keyboard')
        self.get_logger().info('W - Forward, S - Backward, A - Turn Left, D - Turn Right')

    def cmd_vel_callback(self, msg):
        # LIMIT THE SPEED VALUES to reasonable ranges
        linear_x = max(min(msg.linear.x, 2.0), -2.0)   # Limit to ±2 m/s
        angular_z = max(min(msg.angular.z, 4.0), -4.0) # Limit to ±4 rad/s
        
        # Tank drive equations
        wheel_separation = 0.16  # Distance between tracks in meters
        wheel_radius = 0.04     # Wheel radius in meters
        
        # Calculate wheel linear velocities
        left_linear_vel = linear_x - (angular_z * wheel_separation / 2.0)
        right_linear_vel = linear_x + (angular_z * wheel_separation / 2.0)
        
        # Convert to angular velocities (rad/s) and limit
        max_wheel_vel = 20.0  # Maximum wheel angular velocity
        self.left_wheel_vel = max(min(left_linear_vel / wheel_radius, max_wheel_vel), -max_wheel_vel)
        self.right_wheel_vel = max(min(right_linear_vel / wheel_radius, max_wheel_vel), -max_wheel_vel)
        
        # Update positions based on velocities
        time_step = 0.1  # seconds
        self.left_wheel_pos += self.left_wheel_vel * time_step
        self.right_wheel_pos += self.right_wheel_vel * time_step
        
        # Display velocities and movement info
        movement = self.get_movement_description(linear_x, angular_z)
        self.get_logger().info(f'🚀 {movement}')
        self.get_logger().info(f'   Linear: {linear_x:.2f} m/s, Angular: {angular_z:.2f} rad/s')
        self.get_logger().info(f'   Left Wheel: {self.left_wheel_vel:.2f} rad/s, Right Wheel: {self.right_wheel_vel:.2f} rad/s')

    def get_movement_description(self, linear, angular):
        if linear > 0.1 and abs(angular) < 0.1:
            return "MOVING FORWARD"
        elif linear < -0.1 and abs(angular) < 0.1:
            return "MOVING BACKWARD"
        elif angular > 0.1 and abs(linear) < 0.1:
            return "TURNING LEFT"
        elif angular < -0.1 and abs(linear) < 0.1:
            return "TURNING RIGHT"
        elif linear > 0.1 and angular > 0.1:
            return "MOVING FORWARD + TURNING LEFT"
        elif linear > 0.1 and angular < -0.1:
            return "MOVING FORWARD + TURNING RIGHT"
        elif linear < -0.1 and angular > 0.1:
            return "MOVING BACKWARD + TURNING LEFT"
        elif linear < -0.1 and angular < -0.1:
            return "MOVING BACKWARD + TURNING RIGHT"
        else:
            return "STOPPED"

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'chassis'
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [self.left_wheel_vel, self.right_wheel_vel]
        joint_state.effort = []
        
        self.joint_pub.publish(joint_state)

def main():
    rclpy.init()
    node = TankController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Controller stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
