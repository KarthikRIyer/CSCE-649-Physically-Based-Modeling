import bpy

# Specify the name of the collection
collection_name = "Imported_PLY_Files"

# Get the geometry node group
node_group_name = "Geometry Nodes"
node_group = bpy.data.node_groups.get(node_group_name)

if node_group is None:
    print(f"Node group '{node_group_name}' not found.")
else:
    # Get the collection
    collection = bpy.data.collections.get(collection_name)

    if collection is None:
        print(f"Collection '{collection_name}' not found.")
    else:
        # Iterate through all objects in the collection
        for obj in collection.objects:
            if obj.type == 'MESH':
                # Add a Geometry Nodes modifier
                modifier = obj.modifiers.new(name="GeometryNodes", type='NODES')
                modifier.node_group = node_group
                print(f"Applied '{node_group_name}' to '{obj.name}'.")

print("Script completed.")
