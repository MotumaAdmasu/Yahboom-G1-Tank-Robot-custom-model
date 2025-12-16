#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class CorrectWheelController(Node):
    def __init__(self):
        super().__init__('correct_wheel_controller')
        
        # Subscribe to cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0  
        self.yaw = 0.0
        
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        
        # Current velocities
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Timer for continuous updates
        self.timer = self.create_timer(0.05, self.update)  # 20 Hz
        
        self.get_logger().info('🎯 CORRECT WHEEL CONTROLLER READY')
        self.get_logger().info('📝 Use these keys in teleop_twist_keyboard:')
        self.get_logger().info('   I - Move FORWARD (wheels rotate forward)')
        self.get_logger().info('   , - Move BACKWARD (wheels rotate backward)') 
        self.get_logger().info('   J - Turn LEFT')
        self.get_logger().info('   L - Turn RIGHT')
        self.get_logger().info('   K - STOP')

    def cmd_vel_callback(self, msg):
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z
        
        # Log what's happening
        if abs(msg.linear.x) > 0.1:
            direction = "FORWARD" if msg.linear.x > 0 else "BACKWARD"
            self.get_logger().info(f'🚀 MOVING {direction}: {msg.linear.x:.2f} m/s')
        elif abs(msg.angular.z) > 0.1:
            direction = "LEFT" if msg.angular.z > 0 else "RIGHT" 
            self.get_logger().info(f'🔄 TURNING {direction}: {msg.angular.z:.2f} rad/s')
        else:
            self.get_logger().info('⏹️  STOPPED')

    def update(self):
        # Update robot position using velocity
        dt = 0.05  # 20 Hz
        
        # Update orientation
        self.yaw += self.current_angular * dt
        
        # Update position
        self.x += self.current_linear * math.cos(self.yaw) * dt
        self.y += self.current_linear * math.sin(self.yaw) * dt
        
        # Update wheel positions with CORRECTED directions
        wheel_radius = 0.04
        wheel_separation = 0.16
        
        # For forward movement: both wheels should rotate forward (positive values)
        # For backward movement: both wheels should rotate backward (negative values)
        # For turning: wheels rotate in opposite directions
        
        left_wheel_vel = (self.current_linear - (self.current_angular * wheel_separation / 2)) / wheel_radius
        right_wheel_vel = (self.current_linear + (self.current_angular * wheel_separation / 2)) / wheel_radius
        
        # Update wheel positions (positive = forward rotation)
        self.left_wheel_pos += left_wheel_vel * dt
        self.right_wheel_pos += right_wheel_vel * dt
        
        # Publish TF for robot movement
        self.publish_robot_tf()
        
        # Publish joint states for wheel rotation
        self.publish_joint_states()

    def publish_robot_tf(self):
        # Publish transform from odom to base_footprint
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert yaw to quaternion
        t.transform.rotation.w = math.cos(self.yaw / 2)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2)
        
        self.tf_broadcaster.sendTransform(t)

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = []
        joint_state.effort = []
        
        self.joint_pub.publish(joint_state)

def main():
    rclpy.init()
    node = CorrectWheelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Controller stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
