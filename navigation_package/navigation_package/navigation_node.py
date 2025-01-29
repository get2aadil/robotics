#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from collections import deque
import json
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Subscriptions
        self.subscription_result = self.create_subscription(
            String,
            'detection_results',
            self.result_callback,
            10)
        
        self.subscription_position = self.create_subscription(
            Float32,
            'object_position',
            self.position_callback,
            10)

        self.subscription_object = self.create_subscription(
            String,
            'nlp_output',
            self.nlp_callback,
            10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.navigation_pub = self.create_publisher(String, 'navigation_status', 10)
        
        # Initialize variables
        self.object_detected = False
        self.object_of_interest = ""
        self.action = None
        self.object_position = 0.0  # Normalized position [-1, 1]
        self.state = 'IDLE'
        self.position_history = deque(maxlen=5)
        self.last_known_position = 0.0
        self.searching_timer = 0
        # self.basic_action = None
        # self.rotation_angle = 0.0
        # self.rotation_count = 0

        # Initialize flags
        self.received_detection = False
        self.received_position = False

        # Parameters 
        self.declare_parameters(
                namespace='',
                parameters=[
                    ('linear_speed', 0.2),
                    ('angular_speed', 0.3),
                    ('searching_angular_speed', 0.2),
                    ('angular_gain', 0.4),
                    ('max_angular_speed', 0.3),
                    ('position_filter_size', 5),
                    ('searching_timeout', 5),
                ]
            )

        # Get Parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.searching_angular_speed = self.get_parameter('searching_angular_speed').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.searching_timeout = self.get_parameter('searching_timeout').value

        # Initialize rotation vars
        self.rotation_angle = 0.0
        self.rotation_count = 0

        # Timer for control loop
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)
                    
    def result_callback(self, msg):

        # if self.state == 'IDLE':
        #     return

        self.received_detection = True

        if msg.data == "object_found":
            if not self.object_detected:
                self.get_logger().info("Object found, Switching to TRACKING")
            self.object_detected = True
            self.state = 'TRACKING'
            self.rotation_angle = 0.0
            self.rotation_count = 0

        elif msg.data == "object_reached":
            self.get_logger().info("Object reached. Switch to STOPPED")
            self.object_detected = False
            self.state = 'STOPPED'

        else:
            if self.object_detected:
                self.get_logger().info("Object lost, Switching to SEARCHING")
            else:
                self.get_logger().info("No Object detected yet, Switching to SEARCHING")
            self.object_detected = False
            self.state = 'SEARCHING'
            self.position_history.clear()
            self.searching_timer = 0
            # self.last_known_position = 0.0
            # self.rotation_angle = 0.0
            # self.rotation_count = 0

    def position_callback(self, msg):
        # if self.state == 'IDLE':
        #     return

        self.received_position = True
        if self.object_detected:
            self.position_history.append(msg.data)
            self.object_position = sum(self.position_history) / len(self.position_history)
            self.last_known_position = self.object_position

    def nlp_callback(self, msg):
        try: 
            nlp_data = json.loads(msg.data)

            action = nlp_data.get('action','').lower() if nlp_data.get('action') else 'stop'
            object_of_interest = nlp_data.get('object','').lower() if nlp_data.get('object') else ''
            self.get_logger().info(f"Received Action: {action}, object: {object_of_interest}")

            if action == 'stop':
                self.get_logger().info("Received Stop command.")
                self.action = action
                self.object_of_interest = None
                self.state = 'STOPPED'
                # self.basic_action = None
                self.stop_robot()
            
            elif action in ['move_forward','turn_left','turn_right']:
                self.get_logger().info(f"Received basic action: {action}")
                self.action = action
                self.object_of_interest = None
                self.state = 'BASIC_ACTION'
            
            elif action in ['go' , 'search']:
                if object_of_interest != self.object_of_interest:
                    self.get_logger().info(f"New object of interest: {object_of_interest}")
                    self.action = action
                    self.object_of_interest = object_of_interest
                    self.state = 'SEARCHING'
                    # self.basic_action = None
                    self.object_detected = False
                    self.position_history.clear()
                    self.last_known_position = 0.0
                    self.searching_timer = 0
                    self.received_detection = False
                    self.received_position = False
        
            else:
                self.get_logger().info(f"Unknown action received: {action}")
        
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON received in NLP output")
    


        # if msg.data.lower() != self.object_of_interest:
        #     self.get_logger().info(f"Received new object of interest: {msg.data}")
        #     self.state = 'SEARCHING'
        #     self.object_detected = False
        #     self.object_of_interest = msg.data.lower()
        #     self.position_history.clear()
        #     self.last_known_position = 0.0
        #     self.rotation_count = 0
        #     self.rotation_angle = 0.0
        #     self.searching_timer = 0
        #     self.received_detection = False
        #     self.received_position = False

    def control_loop(self):

        # Check if both messages are being received 
        # if not (self.received_detection and self.received_position):
        #     #self.get_logger().info("Waiting for detection and position")
        #     return
        twist = Twist()

        if self.state == 'IDLE':
            return

        # if self.state in ['TRACKING', 'SEARCHING']:
        #     if not (self.received_detection and self.received_position):
        #         return
        

        if self.state == 'TRACKING':

            # Proportional control for angular speed
            angular_z = -self.angular_gain * self.object_position
            angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)

            # Adjust linear speed based on object centering
            linear_x = self.linear_speed * (1 - abs(self.object_position))

            twist.linear.x = linear_x
            twist.angular.z = angular_z

            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"TRACKING: linear_x={twist.linear.x:.2f}, angular_z={twist.angular.z:.2f}, position={self.object_position:.2f}")

        elif self.state == 'SEARCHING':

            self.searching_timer += 1
            if self.searching_timer < self.searching_timeout and self.last_known_position != 0.0:
                # Move towards last known position
                angular_z = -self.angular_gain * self.last_known_position
                angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)
                
                twist.angular.z = angular_z
                twist.linear.x = self.linear_speed * 0.1 # To slow it

                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"SEARCHING: Rotating towards the last known position")
            
            else: 
                angular_speed = self.searching_angular_speed
                twist.angular.z = angular_speed
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"SEARCHING: Rotating to look for the object")

                # Update rotation angle based on time and angular speed
                time_step = self.timer_period  # seconds
                rotation_increment = abs(angular_speed) * time_step  # radians
                self.rotation_angle += rotation_increment

                if self.rotation_angle >= 2 * math.pi:
                    self.rotation_count += 1
                    self.rotation_angle = 0.0
                    self.get_logger().info(f"Completed {self.rotation_count} full rotation(s)")

                    # Stop after 2 rotations if object not found
                    if self.rotation_count >= 2:
                        self.get_logger().info(f"Object not found after {self.rotation_count} rotations. Stopping.")
                        self.stop_robot()
                        self.state = 'IDLE'
                        # Reset variables
                        self.rotation_count = 0
                        self.last_known_position = 0.0
                        self.received_detection = False
                        self.received_position = False
                        self.object_of_interest = ""  # Clear the object of interest

        elif self.state == 'BASIC_ACTION':

            if self.action == 'move_forward':
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"BASIC_ACTION: Moving forward")

            elif self.action == 'turn_left':
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"BASIC_ACTION: Turning left")
                
            elif self.action == 'turn_right':
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"BASIC_ACTION: Turning right")
            else:
                self.get_logger().info(f"BASIC_ACTION: Unknown action {self.basic_action}")
            
            self.state = 'IDLE'
            self.action = 'STOPPED'

        elif self.state == 'STOPPED':
            msg = String()
            msg.data = "REACHED"
            self.navigation_pub.publish(msg)
            self.get_logger().info(f"Publishing the REACHED message")
            self.object_of_interest = ""
            self.received_detection = False
            self.received_position = False
            self.stop_robot()
            self.state = "IDLE"
            

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Stopping robot.")
        self.state = "IDLE"

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
