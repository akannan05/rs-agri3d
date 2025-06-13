#!/usr/bin/env python3


import open3d as o3d

pcd_path = "dense_filtered_pointcloud.pcd"

# Load the point cloud
pcd = o3d.io.read_point_cloud(pcd_path)

if not pcd.has_points():
    print("Error: No points loaded. Check the PCD path.")
    exit(1)

# Visualizer
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='PCD Viewer', width=1280, height=720)
vis.add_geometry(pcd)

opt = vis.get_render_option()
opt.point_size = 2.0  # Adjust for density
opt.background_color = [0, 0, 0]  # Black background

# Run the visualizer
print("Press 's' to save a screenshot, 'q' to quit.")
vis.run()

vis.destroy_window()

