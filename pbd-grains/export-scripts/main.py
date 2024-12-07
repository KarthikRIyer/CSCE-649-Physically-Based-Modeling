import open3d as o3d
import os
from pathlib import Path

def read_and_export_particle_positions(filename, export_filedir):
    with open(filename, 'r') as file:
        lines = file.readlines()

        current_frame = []
        frame_count = 0
        export_count = 0
        for line in lines[1:]:  # Skip the first line
            line = line.strip()
            if line == "timestep":
                if current_frame:
                    frame_count += 1
                    if frame_count % 5 == 0:
                        export_count += 1
                        export_frame_to_ply(current_frame, export_count, export_filedir)
                current_frame = []
            else:
                parts = line.split()
                if parts:  # Check if parts is not empty and the status is 'a'
                    x, y, z = map(float, parts[0:])
                    current_frame.append((x, z, y))

        if current_frame and frame_count % 5 == 4:  # Export the last frame if it exists and is the fifth frame
            export_count += 1
            export_frame_to_ply(current_frame, export_count, export_filedir)


def export_frame_to_ply(frame, export_count, export_filedir):
    points = []
    for particle in frame:
        x, y, z = particle
        points.append([x, y, z])

    # Create point cloud
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # Save to PLY file
    ply_filename = f'frame_{export_count}.ply'
    ply_filename = export_filedir + ply_filename
    Path(export_filedir).mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(ply_filename, point_cloud)
    print(f"Frame {export_count} saved to {ply_filename}")


# Example usage
filename = 'output-csce.txt'
export_filedir = os.path.splitext(filename)[0] + '/'
read_and_export_particle_positions(filename, export_filedir)
