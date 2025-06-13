#!/usr/bin/env python3

import os
import cv2
import yaml
import numpy as np
import open3d as o3d

def load_extrinsic_matrix(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    T = np.array(data['extrinsic_matrix'], dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError("Extrinsic matrix must be 4x4")
    return T

def load_camera_calibration(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    camera_matrix = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(data['distortion_coefficients']['data'], dtype=np.float64).reshape(1, -1)
    return camera_matrix, dist_coeffs

def load_pointcloud(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    return pcd

def project_lidar_to_image(points_lidar, T_lidar_to_cam, camera_matrix, dist_coeffs):
    n_points = points_lidar.shape[0]
    points_h = np.hstack((points_lidar, np.ones((n_points, 1))))  # (N, 4)
    points_cam_h = points_h @ T_lidar_to_cam.T
    points_cam = points_cam_h[:, :3]

    # Filter out points behind the camera
    mask = points_cam[:, 2] > 0
    points_cam = points_cam[mask]

    # Project to image
    rvec = np.zeros((3, 1), dtype=np.float64)
    tvec = np.zeros((3, 1), dtype=np.float64)
    image_points, _ = cv2.projectPoints(points_cam, rvec, tvec, camera_matrix, dist_coeffs)
    image_points = image_points.reshape(-1, 2)

    return image_points, mask

def main():
    base_path = os.path.expanduser('~/camera_lidar_data')
    extrinsic_yaml = os.path.join(base_path, 'extrinsic_matrix.yaml')
    camera_yaml = os.path.join(base_path, 'camera_intrinsics.yaml')
    pcd_path = os.path.join(base_path, 'pair_000001.pcd')
    img_path = os.path.join(base_path, 'pair_000001_left.png')

    # Load calibration and data
    T_lidar_to_cam = load_extrinsic_matrix(extrinsic_yaml)
    camera_matrix, dist_coeffs = load_camera_calibration(camera_yaml)
    pcd = load_pointcloud(pcd_path)
    points_lidar = np.asarray(pcd.points)
    img = cv2.imread(img_path)

    if img is None:
        raise RuntimeError("Failed to load image from: " + img_path)

    h, w = img.shape[:2]

    image_points, mask = project_lidar_to_image(points_lidar, T_lidar_to_cam, camera_matrix, dist_coeffs)
    visible_points = points_lidar[mask]

    # Colorize point cloud
    colors = []
    for (u, v) in image_points:
        x = int(round(u))
        y = int(round(v))
        if 0 <= x < w and 0 <= y < h:
            bgr = img[y, x]
            rgb = [bgr[2]/255.0, bgr[1]/255.0, bgr[0]/255.0]
            cv2.circle(img, (x, y), 2, (0, 255, 0), -1)
        else:
            rgb = [0.0, 0.0, 0.0]
        colors.append(rgb)

    # Create new colorized point cloud
    colorized_pcd = o3d.geometry.PointCloud()
    colorized_pcd.points = o3d.utility.Vector3dVector(visible_points)
    colorized_pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

    # Save colorized point cloud
    output_path = os.path.join(base_path, 'pair_000001_colorized.pcd')
    o3d.io.write_point_cloud(output_path, colorized_pcd)
    print(f"[INFO] Saved colorized point cloud to: {output_path}")

    # Display overlay (optional)
    cv2.imshow('Projected Lidar on Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

