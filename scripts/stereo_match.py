#!/usr/bin/env python3

import cv2
import numpy as np
import os

def stereo_match(left_img_path, right_img_path, output_path='stereo_disparity.png'):
    # Load grayscale images
    left = cv2.imread(left_img_path, cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(right_img_path, cv2.IMREAD_GRAYSCALE)

    if left is None or right is None:
        raise FileNotFoundError("Could not load left or right image.")

    # Set SGBM parameters
    window_size = 5
    min_disp = 0
    num_disp = 128  # must be divisible by 16
    block_size = window_size
    uniquenessRatio = 5
    speckleWindowSize = 50
    speckleRange = 1
    disp12MaxDiff = 1
    P1 = 8 * 1 * window_size ** 2
    P2 = 32 * 1 * window_size ** 2

    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=block_size,
        uniquenessRatio=uniquenessRatio,
        speckleWindowSize=speckleWindowSize,
        speckleRange=speckleRange,
        disp12MaxDiff=disp12MaxDiff,
        P1=P1,
        P2=P2
    )

    print("[INFO] Computing disparity...")
    disparity = stereo.compute(left, right).astype(np.float32) / 16.0

    # Normalize for visualization
    disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
    disp_vis = np.uint8(disp_vis)

    cv2.imwrite(output_path, disp_vis)
    print(f"[DONE] Disparity map saved to: {output_path}")

    return disparity

if __name__ == '__main__':
    base_path = os.path.expanduser('~/camera_lidar_data')
    left_img = os.path.join(base_path, 'pair_000001_left.png')
    right_img = os.path.join(base_path, 'pair_000001_right.png')
    output_disp = os.path.join(base_path, 'stereo_disparity.png')

    stereo_match(left_img, right_img, output_disp)

