#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
import math
from tf2_ros import TransformBroadcaster

class MobileTankController(Node):
    def __init__(self):
        super().__init__('mobile_tank_controller')
        
        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publisher for joint states (wheel rotation)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # TF broadcaster to move the entire robot
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Robot state
        self.x = 0.0  # position in meters
        self.y = 0.0
        self.yaw = 0.0  # orientation in radians
        
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        
        # Timer to update position and publish TF
        self.timer = self.create_timer(0.1, self.update_robot_position)
        
        self.get_logger().info('=== MOBILE TANK CONTROLLER READY ===')
        self.get_logger().info('Use: ros2 run teleop_twist_keyboard teleop_twist_keyboard')
        self.get_logger().info('W - Forward, S - Backward, A - Turn Left, D - Turn Right')

    def cmd_vel_callback(self, msg):
        # Log ALL commands to see what's happening
        self.get_logger().info(f'📡 RECEIVED: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')
        
        # Store the latest velocity command
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def update_robot_position(self):
        # Update robot position based on velocity (simple kinematics)
        dt = 0.1  # time step in seconds
        
        # Update orientation
        self.yaw += self.angular_vel * dt
        
        # Update position
        self.x += self.linear_vel * math.cos(self.yaw) * dt
        self.y += self.linear_vel * math.sin(self.yaw) * dt
        
        # Update wheel positions (for visual feedback)
        wheel_radius = 0.04
        self.left_wheel_pos += (self.linear_vel - (self.angular_vel * 0.16 / 2)) / wheel_radius * dt
        self.right_wheel_pos += (self.linear_vel + (self.angular_vel * 0.16 / 2)) / wheel_radius * dt
        
        # Publish TF transform to move the entire robot
        self.publish_tf_transform()
        
        # Publish joint states for wheel rotation
        self.publish_joint_states()
        
        # Log position occasionally
        self.get_logger().info(f'🤖 POSITION: x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.1f}°', 
                              throttle_duration_sec=1.0)

    def publish_tf_transform(self):
        # This moves the entire robot in RViz!
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # World frame
        t.child_frame_id = 'base_footprint'  # Robot base frame
        
        # Set position
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Set orientation (convert yaw to quaternion)
        from geometry_msgs.msg import Quaternion
        t.transform.rotation = self.yaw_to_quaternion(self.yaw)
        
        self.tf_broadcaster.sendTransform(t)

    def yaw_to_quaternion(self, yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q

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
    node = MobileTankController()
    
    # Initialize velocities
    node.linear_vel = 0.0
    node.angular_vel = 0.0
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Controller stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
