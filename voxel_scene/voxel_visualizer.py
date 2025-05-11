import numpy as np
import open3d as o3d
import json
import os

# Output folders
os.makedirs("../outputs/depth_maps", exist_ok=True)
os.makedirs("../outputs/rgb_views", exist_ok=True)
os.makedirs("../outputs/poses", exist_ok=True)

# Scene: 32 x 32 x 16
scene_size = (32, 32, 16)
voxels = np.zeros(scene_size)

# Ground, wall, and tower
voxels[:, :, 0] = 1
voxels[10:20, 15, 1:4] = 1
voxels[5, 5, 1:10] = 1

# Convert to mesh
def voxels_to_open3d_mesh(voxel_grid):
    mesh = o3d.geometry.TriangleMesh()
    for x in range(voxel_grid.shape[0]):
        for y in range(voxel_grid.shape[1]):
            for z in range(voxel_grid.shape[2]):
                if voxel_grid[x, y, z] == 1:
                    cube = o3d.geometry.TriangleMesh.create_box(1, 1, 1)
                    cube.translate((x, y, z))
                    cube.paint_uniform_color([0.7, 0.7, 0.7])
                    mesh += cube
    mesh.compute_vertex_normals()
    return mesh

mesh = voxels_to_open3d_mesh(voxels)
o3d.io.write_triangle_mesh("../outputs/scene_mesh.ply", mesh)

# Camera poses (aligned with final render view)
camera_poses = [
    {"id": 0, "position": [100, -20, 5], "target": [16, 16, 2]},
    {"id": 1, "position": [30, 10, 40], "target": [16, 16, 2]},
    {"id": 2, "position": [100, -20, 5], "target": [16, 16, 2]}
]
# Save pose metadata
with open("../outputs/poses/camera_poses.json", "w") as f:
    json.dump(camera_poses, f, indent=2)

# Visualize
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Voxel Viewer", width=1920, height=1080)
vis.add_geometry(mesh)
ctr = vis.get_view_control()

# Camera config
pose = camera_poses[0]
eye = np.array(pose["position"], dtype=np.float64)
target = np.array(pose["target"], dtype=np.float64)
up = np.array([0, 0, 1], dtype=np.float64)

front = (target - eye)
front /= np.linalg.norm(front)
ctr.set_lookat(target.tolist())
ctr.set_front(front.tolist())
ctr.set_up(up.tolist())
ctr.set_zoom(0.8)

# Show
vis.run()
vis.destroy_window()
print("Voxel scene created and aligned camera poses saved.")
