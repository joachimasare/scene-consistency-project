# camera_pose_capturer.py
# Load a mesh, allow interactive panning, then save the final camera pose to camera_poses.json

import open3d as o3d
import json

# Load the mesh
mesh = o3d.io.read_triangle_mesh("../outputs/scene_mesh.ply")
mesh.compute_vertex_normals()

# Start visualizer
vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window(window_name="Adjust Camera and Press 'P' to Save Pose")
vis.add_geometry(mesh)

# Define callback to save current camera pose to camera_poses.json
def save_camera_pose(vis):
    ctr = vis.get_view_control()
    params = ctr.convert_to_pinhole_camera_parameters()
    extrinsic = params.extrinsic

    # Extract camera position (translation part of extrinsic)
    position = extrinsic[:3, 3].tolist()
    target = [16, 16, 4]  # Always target center of scene

    # Format to match existing system
    camera_poses = [
        {"id": 0, "position": position, "target": target},
        {"id": 1, "position": position, "target": target},
        {"id": 2, "position": position, "target": target}
    ]

    with open("../outputs/poses/camera_poses.json", "w") as f:
        json.dump(camera_poses, f, indent=2)

    print("✅ Camera pose saved to ../outputs/poses/camera_poses.json")
    return False  # stop capturing after first press

# Register 'P' key (ASCII code 80) to save pose
vis.register_key_callback(ord("P"), save_camera_pose)

print("🖱️ Adjust the camera, then press 'P' to save pose to JSON")
vis.run()
vis.destroy_window()
