#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.bridge = CvBridge()
        self.image_count = 0
        self.output_dir = os.path.expanduser('~/poses_data')

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        self.subscription = self.create_subscription(
            Image,
            '/left/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Subscribed to /left/image_raw and saving images to ~/poses_data')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        filename = os.path.join(self.output_dir, f'image_{self.image_count:06d}.png')
        success = cv2.imwrite(filename, cv_image)

        if success:
            self.get_logger().info(f'Saved {filename}')
            self.image_count += 1
        else:
            self.get_logger().error(f'Failed to save {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

