import bpy
import os
import re

# Path to the directory containing the PLY files
ply_directory = "/Users/karthik/Desktop/CSCE-649-PBM/ply/output-text-hi"

# Create a new collection
collection_name = "Imported_PLY_Files"
new_collection = bpy.data.collections.new(collection_name)
bpy.context.scene.collection.children.link(new_collection)


# Function to extract frame number from file name
def extract_frame_number(file_name):
    match = re.search(r'frame_(\d+)\.ply', file_name)
    return int(match.group(1)) if match else None


# Get a sorted list of PLY files based on frame number
ply_files = sorted([f for f in os.listdir(ply_directory) if f.endswith('.ply')], key=extract_frame_number)

# Import each PLY file and set visibility per frame
for ply_file in ply_files:
    frame_number = extract_frame_number(ply_file)
    if frame_number is None:
        continue

    file_path = os.path.join(ply_directory, ply_file)

    # Import the PLY file
    bpy.ops.import_mesh.ply(filepath=file_path)

    # Get the imported object
    imported_object = bpy.context.selected_objects[0]

    # Link the object to the new collection
    new_collection.objects.link(imported_object)
    bpy.context.scene.collection.objects.unlink(imported_object)

    # Set visibility keyframes
    imported_object.hide_render = True
    imported_object.hide_viewport = True
    imported_object.keyframe_insert(data_path="hide_render", frame=frame_number - 1)
    imported_object.keyframe_insert(data_path="hide_viewport", frame=frame_number - 1)

    imported_object.hide_render = False
    imported_object.hide_viewport = False
    imported_object.keyframe_insert(data_path="hide_render", frame=frame_number)
    imported_object.keyframe_insert(data_path="hide_viewport", frame=frame_number)

    imported_object.hide_render = True
    imported_object.hide_viewport = True
    imported_object.keyframe_insert(data_path="hide_render", frame=frame_number + 1)
    imported_object.keyframe_insert(data_path="hide_viewport", frame=frame_number + 1)

print("PLY files imported and visibility set per frame.")
