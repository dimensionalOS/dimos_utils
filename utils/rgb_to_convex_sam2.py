from segment_anything import sam_model_registry, SamAutomaticMaskGenerator
import lcm
import threading
from lcm_msgs.sensor_msgs import Image
import numpy as np
import torch
import cv2
import time
import sys
from open3d_depth_to_pointcloud import Open3DDepthToPointcloudConverter
import open3d as o3d
from open3d import visualization
from open3d import geometry

class RGBToConvex:
    def __init__(self):
        # === LCM setup ===
        self.lcm = lcm.LCM()
        self.lcm.subscribe("head_cam_rgb#sensor_msgs.Image", self.rgb_callback)
        self.lcm.subscribe("head_cam_depth#sensor_msgs.Image", self.depth_callback)

        self.thread = threading.Thread(target=self._run_lcm)
        self.thread.daemon = True
        self.thread.start()

        self.lock = threading.Lock()

        # === SAM2 setup ===
        model_type = "vit_h"
        sam_checkpoint = "sam_vit_h.pth"
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        sam.to(self.device)
        self.mask_generator = SamAutomaticMaskGenerator(sam)
        print("[SAM2] Model loaded on", self.device)

        self.rgb_image = None
        self.depth_image = None
        self.processed = False

    def _run_lcm(self):
        while True:
            try:
                self.lcm.handle()
            except Exception as e:
                print(f"[LCM Error] {e}")

    def rgb_callback(self, channel, data):
        img_msg = Image.decode(data)
        if img_msg.encoding in ["rgb8", "bgr8"]:
            rgb_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape((img_msg.height, img_msg.width, 3))
            if img_msg.encoding == "bgr8":
                rgb_img = rgb_img[:, :, ::-1]
            with self.lock:
                self.rgb_image = rgb_img
        else:
            print(f"Unsupported RGB encoding: {img_msg.encoding}")

    def depth_callback(self, channel, data):
        img_msg = Image.decode(data)
        if img_msg.encoding == "32FC1":
            depth_img = np.frombuffer(img_msg.data, dtype=np.float32).reshape((img_msg.height, img_msg.width))
        elif img_msg.encoding == "mono16":
            depth_img = np.frombuffer(img_msg.data, dtype=np.uint16).reshape((img_msg.height, img_msg.width)) / 1000.0
        else:
            print(f"Unsupported depth encoding: {img_msg.encoding}")
            return
        with self.lock:
            self.depth_image = depth_img

    def get_segmentation_masks(self):
        with self.lock:
            if self.rgb_image is None or self.depth_image is None:
                print("Segmentation input not available yet.")
                return None

            if self.processed:
                print("Segmentation already processed.")
                return None

            try:
                print("[SAM2] Running automatic mask generation...")
                masks = self.mask_generator.generate(self.rgb_image)

                # === Filter for blue cup and green bottle ===
                target_colors = [
                    np.array([9, 5, 76]),     # blue cup variant
                    np.array([91, 97, 60]),  # green bottle variant
                    np.array([90, 105, 14])
                ]

                tolerance = 20  # you can try 20–30
                filtered = self.get_filtered_masks(self.rgb_image, masks, target_colors, tolerance)

                self.processed = True
                print(f"[SAM2] Found {len(filtered)} relevant masks.")
                return filtered

            except Exception as e:
                print(f"[SAM2 Error] {e}")
                return None

    @staticmethod
    def get_filtered_masks(image, masks, target_colors, tolerance=40):
        selected_masks = []
        for mask in masks:
            seg = mask["segmentation"]  # binary (H, W)

            mean_color = image[seg].mean(axis=0)  # [R, G, B]
            # print(f"Mask mean color: {mean_color}")

            if seg.sum() == 0 or mask['area'] > 12500 or mask['area'] < 500:
                continue

            for target_color in target_colors:
                if np.all(np.abs(mean_color - target_color) < tolerance):
                    selected_masks.append(mask["segmentation"])
                    break
        return selected_masks


def main():
    rgb_to_convex = RGBToConvex()
    print("RGB to Convex converter initialized.")

    try:
        o3dpc = Open3DDepthToPointcloudConverter()
        while not rgb_to_convex.processed:
            if not o3dpc.camera_info_received:
                print("[WAIT] Waiting for camera intrinsics...")
                time.sleep(0.2)
                continue
            masks = rgb_to_convex.get_segmentation_masks()
            if masks is not None:
                rgb_img = rgb_to_convex.rgb_image.copy()
                overlay = rgb_img.copy()
                for mask in masks[:2]:
                    kernel = np.ones((5, 5), np.uint8)
                    cleaned_mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
                    depth_masked = np.where(cleaned_mask, rgb_to_convex.depth_image, 0)

                    # masked_depth = rgb_to_convex.depth_image[mask.astype(bool)]
                    # print(f"Masked depth stats — min: {masked_depth.min()}, max: {masked_depth.max()}, nonzero: {np.count_nonzero(masked_depth)}")

                    # 2. Convert to point cloud
                    pcd = o3dpc.open3d_depth_to_pointcloud(depth_masked)
                    if pcd is None or len(pcd) == 0:
                        print("[ERROR] Empty or invalid point cloud. Skipping visualization.")
                        return
                    pcd_o3d = o3d.geometry.PointCloud()
                    pcd_o3d.points = o3d.utility.Vector3dVector(pcd)  # pcd is your Nx3 numpy array
                    cl, index = pcd_o3d.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                    inlier_cloud = pcd_o3d.select_by_index(index)
                    outlier_cloud = pcd_o3d.select_by_index(index, invert=True)
                    outlier_cloud.paint_uniform_color([1, 0, 0]) # red
                    inlier_cloud.paint_uniform_color([0, 1, 0]) # green
                    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


                    # Assume pcd_o3d is your Open3D PointCloud object (already populated)
                    print(f"[INFO] Point cloud has {np.asarray(pcd_o3d.points).shape[0]} points")

                    # Compute convex hull
                    hull_mesh, hull_indices = inlier_cloud.compute_convex_hull()
                    hull_mesh.compute_vertex_normals()  # optional, for better visualization
                    # Optional: Simplify for display
                    hull_mesh = hull_mesh.simplify_vertex_clustering(voxel_size=0.005)

                    # Color the hull for clarity
                    hull_mesh.paint_uniform_color([1.0, 0.0, 0.0])  # red

                    # Show both point cloud and its convex hull
                    o3d.visualization.draw_geometries(
                        [inlier_cloud, hull_mesh],
                        window_name="Convex Hull Mesh",
                        width=1280,
                        height=720
                    )
                    
                    color = np.random.randint(0, 255, size=(3,), dtype=np.uint8)
                    overlay[mask.astype(bool)] = overlay[mask.astype(bool)] * 0.5 + color * 0.5

                overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)
                output_path = "sam2_overlay_output.png"
                cv2.imwrite(output_path, overlay_bgr)
                print(f"[INFO] Saved overlay visualization to: {output_path}")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Exiting...")
        sys.exit(0)
        rgb_to_convex.thread.join(timeout=1.0)

if __name__ == "__main__":
    main()
