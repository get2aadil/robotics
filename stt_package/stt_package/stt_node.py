#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import os
import sys
import json
import pyaudio
from vosk import Model, KaldiRecognizer
from ament_index_python.packages import get_package_share_directory

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        self.publisher_ = self.create_publisher(String, 'voice_commands', 10)
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)
        
        # Use get_package_share_directory to find the model path
        package_share_directory = get_package_share_directory('stt_package')
        #model_path = os.path.join(package_share_directory, 'models', 'vosk-model-small-en-us-0.15')
        model_path = os.path.join(package_share_directory, 'models', 'vosk-model-small-en-in-0.4')

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model path '{model_path}' does not exist")
            sys.exit(1)

        self.get_logger().info(f"Using model at '{model_path}'")
        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)
        self.wake_word = "okay buddy"

        self.audio_stream = self.setup_audio_stream()
        self.get_logger().info("Please start speaking...")

        self.process_audio_stream()

    def setup_audio_stream(self):
        p = pyaudio.PyAudio()

        #device_index = 0
        #for i in range(p.get_device_count()):
        #    info = p.get_device_info_by_index(i)
        #    if "USB PnP" in info['name']:
        #        device_index = i
        #    self.get_logger().info(f"Device index: {device_index}")
        #    self.get_logger().info(f"Device {i}: {info['name']}")
        
        # device_index = 3

        stream = p.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=16000,
                        input=True,
                        frames_per_buffer=8000,
                        )
                        #input_device_index=device_index)
        return stream

    def process_audio_stream(self):
        try:
            while rclpy.ok():
                data = self.audio_stream.read(4000, exception_on_overflow=False)
                if self.recognizer.AcceptWaveform(data):
                    result = self.recognizer.Result()
                    result_dict = json.loads(result)
                    recognized_text = result_dict.get('text', '').lower()

                    if self.wake_word in recognized_text:
                        self.get_logger().info("Wake word detected")
                        state_msg = String()
                        state_msg.data = "listening"
                        self.state_publisher.publish(state_msg)
                        self.listen_for_command()
                else:
                    # partial_result = self.recognizer.PartialResult()
                    # partial_dict = json.loads(partial_result)
                    # print("Partial result: ", partial_dict.get('partial', ''))
                    pass  # Optionally handle partial results
        except Exception as e:
            self.get_logger().error(f"Error in process_audio_stream: {e}")
        finally:
            self.audio_stream.stop_stream()
            self.audio_stream.close()

    def listen_for_command(self):
        self.get_logger().info("Listening for command...")
        command_text = ""
        try:
            while True:
                data = self.audio_stream.read(4000, exception_on_overflow=False)
                if self.recognizer.AcceptWaveform(data):
                    result = self.recognizer.Result()
                    result_dict = json.loads(result)
                    command_text = result_dict.get('text', '')
                    self.get_logger().info(f"Command: {command_text}")
                    # Publish the command
                    msg = String()
                    msg.data = command_text
                    self.publisher_.publish(msg)
                    state_msg = String()
                    state_msg.data = "idle"
                    self.state_publisher.publish(state_msg)
                    break
                else:
                    pass  # Optionally handle partial results
        except Exception as e:
            self.get_logger().error(f"Error in listen_for_command: {e}")

def main(args=None):
    rclpy.init(args=args)
    stt_node = STTNode()
    try:
        rclpy.spin(stt_node)
    except KeyboardInterrupt:
        pass
    finally:
        stt_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

