#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import os
#import openai
from openai import OpenAI
from dotenv import load_dotenv
import json

class NLPNode(Node):
    def __init__(self):
        super().__init__('nlp_node')
        self.subscription = self.create_subscription(
            String,
            'voice_commands',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'nlp_output', 10)
        load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))
        api_key = os.getenv('OPENAI_API_KEY')
        organization = os.getenv('OPENAI_ORG_ID')

        if not api_key or not organization:
            self.get_logger().error(f"OpenAI API credentials not found.Path of env is: {os.path.join(os.path.dirname(__file__), '.env')}")
            raise RuntimeError("OpenAI API credentials not found.")
        self.client = OpenAI(
                api_key=api_key,
                organization=organization
        )
            

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        nlp_output = self.get_openai_response(command)
        if nlp_output:
            self.get_logger().info(f"NLP Output: {nlp_output}")
            # Publish the object
            msg = String()
            msg.data = nlp_output
            self.publisher_.publish(msg)
        else:
            self.get_logger().error("Failed to process command.")

    def get_openai_response(self, prompt):
        initial_prompt = (
            "You are a command interpreter for a robot. "
            "Extract the action, object, and any attributes (like color, size, etc.) from the following command, "
            "which the robot needs to execute. "
            "Provide the response in the following JSON format: "
            "{\"action\": <action>, \"object\": <object>, \"attributes\": {\"color\": <color>, \"size\": <size>, ...}}. "
            "Possible actions are: 'go', 'search', 'stop', 'move_forward', 'turn_left', 'turn_right'. "
            "If the command is 'stop', 'move_forward', 'turn_left', or 'turn_right', the object and attributes can be null. "
            "Do not include any extra text in your response."
        )
        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": initial_prompt},
                    {"role": "user", "content": prompt}
                ]
            )
            answer = response.choices[0].message.content.strip()
            try:
                json.loads(answer)
                return answer
            except json.JSONDecodeError:
                self.get_logger().error("Invalid JSON received from OpenAI")
                return None
        except Exception as e:
            self.get_logger().error(f"OpenAI API error: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    try:
        nlp_node = NLPNode()
    except RuntimeError as e:
        rclpy.logging.get_logger('nlp_node').error(str(e))
        rclpy.shutdown()
        return
    try:
        rclpy.spin(nlp_node)
    except KeyboardInterrupt:
        pass
    finally:
        nlp_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

