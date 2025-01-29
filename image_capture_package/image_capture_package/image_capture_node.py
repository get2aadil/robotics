#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.publisher_ = self.create_publisher(CompressedImage, 'image_compressed', qos)
        self.cap = cv2.VideoCapture(0)  # Adjust the index if necessary
        # self.bridge = CvBridge()

        # Check if the camera opened successfully
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera.")
            self.destroy_node()
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Create a timer that triggers every 2 seconds
        timer_period = 1.0/30  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Image Capture Node has been started. Capturing and publishing images at 10 fps.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            ret, compressed_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            if ret: 
                image_message = CompressedImage()
                image_message.header.stamp = self.get_clock().now().to_msg()
                image_message.format = 'jpeg'
                image_message.data = np.array(compressed_frame).tobytes()
                self.publisher_.publish(image_message)
            else:
                self.get_logger().error("Failed to compress image")
            # # Publish the image immediately after capturing
            # image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # self.publisher_.publish(image_message)
            # #self.get_logger().info("Captured and published image on /image_raw")
        else:
            self.get_logger().error("Failed to capture image.")

    def destroy(self):
        # Release the camera when shutting down
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_capture_node = ImageCaptureNode()
    try:
        rclpy.spin(image_capture_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_capture_node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

