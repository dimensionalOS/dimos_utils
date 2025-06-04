#!/usr/bin/env python3
"""
RGB-to-Convex Pipeline using SAM2 and Open3D

This script:
  1. Subscribes to RGB and depth LCM streams.
  2. Runs SAM2 to generate segmentation masks.
  3. Filters masks for target objects (blue cup, green bottle).
  4. Converts masked depth to point cloud via Open3D.
  5. Removes statistical outliers from the point cloud.
  6. Computes convex hull meshes and returns them as objects.
"""

import sys
import threading
import time
from typing import List, Optional

import cv2
import lcm
import numpy as np
import open3d as o3d
import torch
from lcm_msgs.sensor_msgs import Image
from open3d_depth_to_pointcloud import Open3DDepthToPointcloudConverter
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry

# ------------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------------

MODEL_TYPE = "vit_h"
SAM_CHECKPOINT = "sam_vit_h.pth"
TARGET_COLORS = [
    np.array([9, 5, 76]),    # Dark blue cup
    np.array([91, 97, 60]),  # Green bottle
    np.array([90, 105, 14])  # Variant green bottle
]
COLOR_TOLERANCE = 20
MIN_MASK_AREA = 500
MAX_MASK_AREA = 12_500

# ------------------------------------------------------------------------------
# Helper Functions
# ------------------------------------------------------------------------------


def filter_masks_by_color(
    image: np.ndarray,
    masks: List[dict],
    target_colors: List[np.ndarray],
    tolerance: int = COLOR_TOLERANCE,
) -> List[np.ndarray]:
    """
    Given an RGB image and a list of SAM2-generated masks,
    return only those mask arrays whose mean RGB falls within
    'tolerance' of any 'target_colors' and whose area is within bounds.
    """
    selected = []

    for mask_info in masks:
        seg = mask_info["segmentation"]
        area = mask_info["area"]

        if area < MIN_MASK_AREA or area > MAX_MASK_AREA:
            continue

        # Compute mean color over mask region
        mean_rgb = image[seg].mean(axis=0)

        # Compare to each target color
        for target in target_colors:
            if np.all(np.abs(mean_rgb - target) < tolerance):
                selected.append(seg)
                break

    return selected


def visualize_inlier_outlier(cloud: o3d.geometry.PointCloud, indices: List[int]) -> None:
    """
    Given an Open3D PointCloud and a list of inlier indices,
    paint inliers gray, outliers red, and visualize both.
    """
    inlier_cloud = cloud.select_by_index(indices)
    outlier_cloud = cloud.select_by_index(indices, invert=True)

    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])  # light gray
    outlier_cloud.paint_uniform_color([1.0, 0.0, 0.0])  # red

    o3d.visualization.draw_geometries(
        [inlier_cloud, outlier_cloud],
        zoom=0.3412,
        front=[0.4257, -0.2125, -0.8795],
        lookat=[2.6172, 2.0475, 1.532],
        up=[-0.0694, -0.9768, 0.2024],
    )


# ------------------------------------------------------------------------------
# Main Classes
# ------------------------------------------------------------------------------


class RGBToConvex:
    """
    Subscribes to RGB and depth images via LCM, runs SAM2 segmentation,
    filters masks by target object color, and returns binary masks.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.rgb_image: Optional[np.ndarray] = None
        self.depth_image: Optional[np.ndarray] = None
        self._processed = False

        # Initialize LCM and subscribe
        self._lcm = lcm.LCM()
        self._lcm.subscribe("head_cam_rgb#sensor_msgs.Image", self._rgb_callback)
        self._lcm.subscribe("head_cam_depth#sensor_msgs.Image", self._depth_callback)

        # Start background thread to handle LCM
        self._lcm_thread = threading.Thread(target=self._run_lcm, daemon=True)
        self._lcm_thread.start()

        # Initialize SAM2
        device = "cuda" if torch.cuda.is_available() else "cpu"
        sam = sam_model_registry[MODEL_TYPE](checkpoint=SAM_CHECKPOINT)
        sam.to(device)
        self._mask_generator = SamAutomaticMaskGenerator(sam)
        print(f"[SAM2] Model loaded on {device}")

    def _run_lcm(self) -> None:
        """Continuously handle incoming LCM messages."""
        while True:
            try:
                self._lcm.handle()
            except Exception as e:
                print(f"[LCM Error] {e}")

    def _rgb_callback(self, channel: str, data: bytes) -> None:
        """LCM callback to decode and store the latest RGB image."""
        msg = Image.decode(data)
        if msg.encoding in ("rgb8", "bgr8"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            if msg.encoding == "bgr8":
                arr = arr[:, :, ::-1]
            with self._lock:
                self.rgb_image = arr
        else:
            print(f"[RGB Callback] Unsupported encoding: {msg.encoding}")

    def _depth_callback(self, channel: str, data: bytes) -> None:
        """LCM callback to decode and store the latest depth image."""
        msg = Image.decode(data)
        if msg.encoding == "32FC1":
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        elif msg.encoding == "mono16":
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)) / 1000.0
        else:
            print(f"[Depth Callback] Unsupported encoding: {msg.encoding}")
            return

        with self._lock:
            self.depth_image = arr

    def get_segmentation_masks(self) -> Optional[List[np.ndarray]]:
        """
        Generate segmentation masks via SAM2, then filter for target objects.
        Returns a list of binary mask arrays or None if not ready/processed.
        """
        with self._lock:
            if self.rgb_image is None or self.depth_image is None:
                print("[SAM2] Waiting for both RGB and depth images.")
                return None

            if self._processed:
                return None

            try:
                print("[SAM2] Running automatic mask generation...")
                raw_masks = self._mask_generator.generate(self.rgb_image)
                filtered = filter_masks_by_color(
                    self.rgb_image, raw_masks, TARGET_COLORS, COLOR_TOLERANCE
                )

                self._processed = True
                print(f"[SAM2] Found {len(filtered)} relevant masks.")
                return filtered

            except Exception as e:
                print(f"[SAM2 Error] {e}")
                return None


# ------------------------------------------------------------------------------
# Entry Point
# ------------------------------------------------------------------------------


def run_pipeline() -> List[o3d.geometry.TriangleMesh]:
    """
    1. Instantiate RGBToConvex to get filtered segmentation masks.
    2. Instantiate Open3DDepthToPointcloudConverter to turn depth into a point cloud.
    3. Remove outliers, compute convex hull, and return a list of meshes.
    """
    rgb_to_convex = RGBToConvex()
    print("[Main] RGB-to-Convex converter initialized.")

    # Wait for camera intrinsics and images, then process once
    converter = Open3DDepthToPointcloudConverter(swap_y_z=True)
    convex_hulls: List[o3d.geometry.TriangleMesh] = []

    try:
        while True:
            # Wait for camera info
            if not converter.camera_info_received:
                time.sleep(0.2)
                continue

            masks = rgb_to_convex.get_segmentation_masks()
            if masks is None:
                time.sleep(0.2)
                continue

            for mask in masks[0:3:2]: # Skip the masks we don't want to process
                # 1. Clean mask with morphological opening
                kernel = np.ones((5, 5), np.uint8)
                cleaned_mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)

                # 2. Extract masked depth (zeros elsewhere)
                with rgb_to_convex._lock:
                    depth = rgb_to_convex.depth_image.copy()
                depth_masked = np.where(cleaned_mask, depth, 0.0)

                # 3. Convert depth to Nx3 numpy array of points
                init_points = converter.open3d_depth_to_pointcloud(depth_masked)
                transform = converter.get_transform("world", "camera_center_link")
                print(f"This is rgb_to_convex pipeline")
                print(f"         Translation = {transform.translation()}")
                quat_before = transform.rotation().ToQuaternion()
                print(f"         Rotation (w, x, y, z) = "
                    f"[{quat_before.w():.6f}, {quat_before.x():.6f}, "
                    f"{quat_before.y():.6f}, {quat_before.z():.6f}]")
    
                points = converter.transform_point_cloud_with_open3d(init_points, transform)
                if points is None or len(points) == 0:
                    print("[Main] Empty or invalid point cloud. Skipping this mask.")
                    continue

                # 4. Create Open3D point cloud object
                pcd_o3d = o3d.geometry.PointCloud()
                pcd_o3d.points = o3d.utility.Vector3dVector(points)

                # 5. Remove statistical outliers
                _, inlier_indices = pcd_o3d.remove_statistical_outlier(
                    nb_neighbors=20, std_ratio=2.0
                )
                inlier_cloud = pcd_o3d.select_by_index(inlier_indices)
                outlier_cloud = pcd_o3d.select_by_index(inlier_indices, invert=True)
                outlier_cloud.paint_uniform_color([1, 0, 0]) # red
                inlier_cloud.paint_uniform_color([0, 1, 0]) # green
                o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

                # 6. Compute convex hull
                hull_mesh, _ = inlier_cloud.compute_convex_hull()
                hull_mesh.compute_vertex_normals()
                hull_mesh = hull_mesh.simplify_vertex_clustering(voxel_size=0.005)

                convex_hulls.append(hull_mesh)

            break  # only process once; remove this if you want continuous looping

    except KeyboardInterrupt:
        print("\n[Main] Exiting gracefully.")
        sys.exit(0)

    return convex_hulls


def main() -> None:
    hulls = run_pipeline()
    for idx, mesh in enumerate(hulls):
        filename = f"convex_hull_{idx}.ply"
        o3d.io.write_triangle_mesh(filename, mesh)
        print(f"[Main] Saved hull mesh to: {filename}")


if __name__ == "__main__":
    main()
