#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class SimpleTankMovement(Node):
    def __init__(self):
        super().__init__('simple_tank_movement')
        
        # Subscribe to cmd_vel topic
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0  
        self.yaw = 0.0
        
        # Current velocities
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Timer for continuous updates
        self.timer = self.create_timer(0.05, self.update)  # 20 Hz
        
        self.get_logger().info('🚀 SIMPLE TANK MOVEMENT READY')
        self.get_logger().info('📝 Use teleop_twist_keyboard with these keys:')
        self.get_logger().info('   I - Move FORWARD')
        self.get_logger().info('   , - Move BACKWARD') 
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
        
        # Publish TF for robot movement
        self.publish_robot_tf()
        
        # Log position occasionally
        self.get_logger().info(f'📍 POSITION: x={self.x:.2f}, y={self.y:.2f}, angle={math.degrees(self.yaw):.1f}°', 
                              throttle_duration_sec=2.0)

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

def main():
    rclpy.init()
    node = SimpleTankMovement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Controller stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
