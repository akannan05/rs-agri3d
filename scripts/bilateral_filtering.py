#!/usr/bin/env python3

import cv2
import numpy as np
import open3d as o3d
import os

def load_camera_intrinsics():
    # Replace with actual values or load from YAML
    fx = 955.475
    fy = 955.865
    cx = 648.185
    cy = 356.526
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]])
    return K, fx, fy, cx, cy

def disparity_to_depth(disparity, fx, baseline):
    disparity = disparity.astype(np.float32)
    disparity[disparity <= 0] = 0.1  # Prevent div-by-zero
    depth = (fx * baseline) / disparity
    return depth

def bilateral_filter_depth(depth, img):
    try:
        import cv2.ximgproc as xip
    except ImportError:
        raise ImportError("cv2.ximgproc module not found. Install opencv-contrib-python.")

    # Ensure depth is float32
    if depth.dtype != np.float32:
        depth = depth.astype(np.float32)

    # Convert image to grayscale and float32
    if img.ndim == 3:
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        img_gray = img

    if img_gray.dtype != np.float32:
        img_gray = img_gray.astype(np.float32)

    # Now both img_gray and depth are float32, 1 channel
    filtered = xip.jointBilateralFilter(
        img_gray,  # guide image (grayscale, float32)
        depth,     # depth image (float32)
        9,         # d
        25,        # sigmaColor (adjust if needed)
        5          # sigmaSpace
    )
    return filtered

def depth_to_pointcloud(depth, rgb_img, K):
    h, w = depth.shape
    i, j = np.meshgrid(np.arange(w), np.arange(h))

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    z = depth
    x = (i - cx) * z / fx
    y = (j - cy) * z / fy

    points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    colors = rgb_img.reshape(-1, 3) / 255.0

    valid = (z > 0).reshape(-1)
    return points[valid], colors[valid]

def main():
    base_path = os.path.expanduser('~/camera_lidar_data')

    rgb_path = os.path.join(base_path, 'pair_000001_left.png')
    disp_path = os.path.join(base_path, 'stereo_disparity.png')

    img = cv2.imread(rgb_path)
    if img is None:
        raise RuntimeError("Failed to load RGB image.")

    disp = cv2.imread(disp_path, cv2.IMREAD_UNCHANGED).astype(np.float32) /256.0
    if disp is None:
        raise RuntimeError("Failed to load disparity image.")

    K, fx, fy, cx, cy = load_camera_intrinsics()

    # Assumptions: baseline in meters, disparity in pixels
    baseline = 0.12  # in meters, typical for KITTI-like stereo rigs
    depth = disparity_to_depth(disp, fx, baseline)

    filtered_depth = bilateral_filter_depth(depth, img)

    points, colors = depth_to_pointcloud(filtered_depth, img, K)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    out_path = os.path.join(base_path, 'dense_filtered_pointcloud.pcd')
    o3d.io.write_point_cloud(out_path, pcd)
    print(f"[INFO] Saved dense filtered point cloud to: {out_path}")

if __name__ == '__main__':
    main()

