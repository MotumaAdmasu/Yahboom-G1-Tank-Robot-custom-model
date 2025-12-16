#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MovementTester(Node):
    def __init__(self):
        super().__init__('movement_tester')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Moving FORWARD: linear=0.5 m/s')
        
    def move_backward(self):
        twist = Twist()
        twist.linear.x = -0.5
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Moving BACKWARD: linear=-0.5 m/s')
        
    def turn_left(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 1.0
        self.publisher.publish(twist)
        self.get_logger().info('Turning LEFT: angular=1.0 rad/s')
        
    def turn_right(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -1.0
        self.publisher.publish(twist)
        self.get_logger().info('Turning RIGHT: angular=-1.0 rad/s')
        
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('STOPPING')

def main():
    rclpy.init()
    node = MovementTester()
    
    try:
        while True:
            print("\n=== TANK MOVEMENT TEST ===")
            print("1: Move Forward")
            print("2: Move Backward") 
            print("3: Turn Left")
            print("4: Turn Right")
            print("5: Stop")
            print("0: Exit")
            
            choice = input("Select movement: ")
            
            if choice == '1':
                node.move_forward()
            elif choice == '2':
                node.move_backward()
            elif choice == '3':
                node.turn_left()
            elif choice == '4':
                node.turn_right()
            elif choice == '5':
                node.stop()
            elif choice == '0':
                break
            else:
                print("Invalid choice")
                
            time.sleep(1)  # Keep the command active for 1 second
            node.stop()    # Stop after each command
            
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
