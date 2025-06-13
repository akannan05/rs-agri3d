#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
import open3d as o3d
import numpy as np
import threading
import sys
import termios
import tty
import os
import cv2
import csv

class SaveData(Node):
    def __init__(self):
        super().__init__('save_data_node')

        self.bridge = CvBridge()
        self.storage_path = os.path.expanduser('~/camera_lidar_data')
        os.makedirs(self.storage_path, exist_ok=True)

        # Topics
        self.left_image_topic = '/left/image_raw'
        self.right_image_topic = '/right/image_raw'
        self.lidar_topic = '/rslidar_points'
        self.imu_topic = '/imu/data_raw'

        # Latest messages
        self.latest_left_image_msg = None
        self.latest_right_image_msg = None
        self.latest_cloud_msg = None
        self.latest_imu_msg = None

        self.pair_count = 1
        self.save_flag = False

        # Subscriptions
        self.create_subscription(Image, self.left_image_topic, self.left_image_callback, 10)
        self.create_subscription(Image, self.right_image_topic, self.right_image_callback, 10)
        self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile)

        self.get_logger().info('SaveData node started. Press `;` to save data pair (left+right+pcd+imu).')
        self.start_key_listener()

    def left_image_callback(self, msg):
        self.latest_left_image_msg = msg

    def right_image_callback(self, msg):
        self.latest_right_image_msg = msg

    def lidar_callback(self, msg):
        self.latest_cloud_msg = msg

    def imu_callback(self, msg):
        self.latest_imu_msg = msg

    def start_key_listener(self):
        def listen():
            while True:
                key = self.get_key()
                if key == ';':
                    self.get_logger().info('Keypress detected: `;` - saving data')
                    self.save_flag = True
                    self.try_save()
        thread = threading.Thread(target=listen, daemon=True)
        thread.start()

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def try_save(self):
        # Basic checks
        if not all([self.latest_left_image_msg, self.latest_right_image_msg, self.latest_cloud_msg, self.latest_imu_msg]):
            self.get_logger().warn('Missing data, skipping save. Make sure all topics are publishing.')
            return

        try:
            # Left image
            left_img = self.bridge.imgmsg_to_cv2(self.latest_left_image_msg, desired_encoding='bgr8')
            left_path = os.path.join(self.storage_path, f'pair_{self.pair_count:06d}_left.png')
            cv2.imwrite(left_path, left_img)
            self.get_logger().info(f'Left image saved to {left_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save left image: {e}')
            return

        try:
            # Right image
            right_img = self.bridge.imgmsg_to_cv2(self.latest_right_image_msg, desired_encoding='bgr8')
            right_path = os.path.join(self.storage_path, f'pair_{self.pair_count:06d}_right.png')
            cv2.imwrite(right_path, right_img)
            self.get_logger().info(f'Right image saved to {right_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save right image: {e}')
            return

        try:
            # Point cloud
            points = []
            for p in point_cloud2.read_points(self.latest_cloud_msg, skip_nans=True):
                points.append([p[0], p[1], p[2]])
            if points:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float32))
                pcd_path = os.path.join(self.storage_path, f'pair_{self.pair_count:06d}.pcd')
                o3d.io.write_point_cloud(pcd_path, pcd)
                self.get_logger().info(f'Point cloud saved to {pcd_path}')
            else:
                self.get_logger().warn('No valid points in point cloud.')
        except Exception as e:
            self.get_logger().error(f'Failed to save point cloud: {e}')
            return

        try:
            # IMU data
            imu_msg = self.latest_imu_msg
            imu_path = os.path.join(self.storage_path, f'pair_{self.pair_count:06d}_imu.csv')
            with open(imu_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                    'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                    'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z'
                ])
                writer.writerow([
                    imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w,
                    imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
                    imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z
                ])
            self.get_logger().info(f'IMU data saved to {imu_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save IMU data: {e}')
            return

        # Successful save
        self.get_logger().info(f'Successfully saved data pair {self.pair_count:06d}')
        self.pair_count += 1
        self.save_flag = False


def main(args=None):
    rclpy.init(args=args)
    node = SaveData()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

