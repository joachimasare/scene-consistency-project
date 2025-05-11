import numpy as np
import open3d as o3d
import open3d.core as o3c
import json
import os

# Paths
MESH_PATH = "../outputs/scene_mesh.ply"
POSES_PATH = "../outputs/poses/camera_poses.json"
DEPTH_DIR = "../outputs/depth_maps"
RGB_DIR = "../outputs/rgb_views"

os.makedirs(DEPTH_DIR, exist_ok=True)
os.makedirs(RGB_DIR, exist_ok=True)

# Load mesh and convert to RaycastingScene
mesh = o3d.io.read_triangle_mesh(MESH_PATH)
mesh.compute_vertex_normals()
scene = o3d.t.geometry.RaycastingScene()
_ = scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))

# Load camera poses
with open(POSES_PATH, 'r') as f:
    camera_poses = json.load(f)

# Intrinsics
img_w, img_h = 640, 480
fov = 60.0
focal = 0.5 * img_w / np.tan(np.deg2rad(fov / 2))
intrinsic_matrix = np.array([
    [focal, 0, img_w / 2],
    [0, focal, img_h / 2],
    [0, 0, 1]
], dtype=np.float64)
intrinsic_tensor = o3c.Tensor(intrinsic_matrix, dtype=o3c.Dtype.Float64)

# Render each pose
for pose in camera_poses:
    cam_id = pose["id"]
    eye = np.array(pose["position"], dtype=np.float64)
    target = np.array(pose["target"], dtype=np.float64)
    up_guess = np.array([0, 0, 1], dtype=np.float64)

    z = eye - target
    z /= np.linalg.norm(z)
    x = np.cross(up_guess, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)

    extrinsic = np.eye(4)
    extrinsic[:3, :3] = np.stack([x, y, z], axis=1)
    extrinsic[:3, 3] = eye
    extrinsic_tensor = o3c.Tensor(extrinsic, dtype=o3c.Dtype.Float64)

    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        intrinsic_tensor, extrinsic_tensor, img_w, img_h
    )

    raycast = scene.cast_rays(rays)
    depth = raycast["t_hit"].numpy().reshape((img_h, img_w))
    depth[depth == np.inf] = 0

    np.save(f"{DEPTH_DIR}/depth_{cam_id}.npy", depth)

    if np.max(depth) > 0:
        vis = (255 * (depth / np.max(depth))).astype(np.uint8)
    else:
        vis = (depth * 255).astype(np.uint8)

    o3d.io.write_image(f"{RGB_DIR}/rgb_{cam_id}.png", o3d.geometry.Image(vis))

print("All views rendered and saved.")
