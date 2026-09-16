import shutil


file = open("/home/pradnesh/Desktop/MMR/Hexapod/hexapod/urdf/hexapod.urdf")
content = file.read()
file.close()

content = content.replace("root", "base_link")
content = content.replace("package://hexapod/meshes/", "file:///home/pradnesh/Desktop/MMR/Hexapod/hexapod/meshes/")
#content = content.replace("package://hexapod/meshes/", "/home/pradnesh/Desktop/Hexapod/hexapod/meshes/")
shutil.copy2("/home/pradnesh/Desktop/MMR/plane.urdf", "/home/pradnesh/Desktop/MMR/Hexapod/hexapod/urdf/")

file = open("/home/pradnesh/Desktop/MMR/Hexapod/hexapod/urdf/hexapod.urdf", "w")
file.write(content)
file.close()

import bpy
import os
from mathutils import Vector

# ==========================
# CONFIGURATION
# ==========================

INPUT_DIR = r"/home/pradnesh/Desktop/MMR/Hexapod/hexapod/meshes"
OUTPUT_DIR = r"/home/pradnesh/Desktop/MMR/Hexapod/hexapod/collision_meshes"

os.makedirs(OUTPUT_DIR, exist_ok=True)

# ==========================
# HELPER FUNCTIONS
# ==========================

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    # Remove orphan meshes
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)

def import_stl(filepath):
    bpy.ops.wm.stl_import(filepath=filepath)
    return bpy.context.selected_objects[0]

def create_bbox(obj):
    # Use world-space bounding box
    corners = [obj.matrix_world @ Vector(corner)
           for corner in obj.bound_box]

    min_x = min(v.x for v in corners)
    min_y = min(v.y for v in corners)
    min_z = min(v.z for v in corners)

    max_x = max(v.x for v in corners)
    max_y = max(v.y for v in corners)
    max_z = max(v.z for v in corners)

    sx = max_x - min_x
    sy = max_y - min_y
    sz = max_z - min_z

    cx = (min_x + max_x) / 2
    cy = (min_y + max_y) / 2
    cz = (min_z + max_z) / 2

    bpy.ops.mesh.primitive_cube_add(location=(cx, cy, cz))
    cube = bpy.context.active_object

    # Blender cube default size = 2
    cube.scale = (sx / 2, sy / 2, sz / 2)

    return cube

def export_stl(obj, filepath):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    bpy.ops.wm.stl_export(
        filepath=filepath,
        export_selected_objects=True
    )

# ==========================
# MAIN LOOP
# ==========================

for filename in os.listdir(INPUT_DIR):
    if not filename.lower().endswith(".stl"):
        continue

    print("Processing:", filename)

    clear_scene()

    mesh = import_stl(os.path.join(INPUT_DIR, filename))

    bpy.context.view_layer.update()

    bbox = create_bbox(mesh)

    export_stl(
        bbox,
        os.path.join(OUTPUT_DIR, filename)
    )

print("Done.")

import xml.etree.ElementTree as ET
import os

def add_collision_tags(input_urdf, output_urdf, mesh_folder_mapping=None):
    """
    Parses a URDF, copies geometry and origin from <visual> to a new <collision> tag.
    
    :param input_urdf: Path to the input URDF file.
    :param output_urdf: Path to save the modified URDF.
    :param mesh_folder_mapping: A tuple of (old_str, new_str) to update the mesh file path.
                                e.g., ('/visual/', '/collision/')
    """
    if not os.path.exists(input_urdf):
        print(f"Error: File {input_urdf} not found.")
        return

    # Register namespaces if any (URDF usually doesn't use them, but good practice)
    ET.register_namespace('', '')
    
    tree = ET.parse(input_urdf)
    root = tree.getroot()

    # Find all link elements
    links = root.findall('link')
    
    modified_count = 0

    for link in links:
        visual = link.find('visual')
        
        # Skip if there's no visual tag to copy from, or if collision already exists
        if visual is None:
            continue
        if link.find('collision') is not None:
            print(f"Skipping link '{link.get('name')}': Collision tag already exists.")
            continue

        # Create the new collision element
        collision = ET.Element('collision')

        # 1. Copy over the origin (position offset and rotation) if it exists
        origin = visual.find('origin')
        if origin is not None:
            # Create a deep-ish copy of the origin element
            coll_origin = ET.Element('origin')
            if 'xyz' in origin.attrib:
                coll_origin.set('xyz', origin.get('xyz'))
            if 'rpy' in origin.attrib:
                coll_origin.set('rpy', origin.get('rpy'))
            collision.append(coll_origin)

        # 2. Copy and update the geometry
        visual_geom = visual.find('geometry')
        if visual_geom is not None:
            coll_geom = ET.Element('geometry')
            mesh = visual_geom.find('mesh')
            
            if mesh is not None:
                coll_mesh = ET.Element('mesh')
                filename = mesh.get('filename')
                
                # Apply path mapping for the collision folder if provided
                if mesh_folder_mapping and filename:
                    old_dir, new_dir = mesh_folder_mapping
                    filename = filename.replace(old_dir, new_dir)
                
                coll_mesh.set('filename', filename)
                
                # Copy scale if it exists
                if 'scale' in mesh.attrib:
                    coll_mesh.set('scale', mesh.get('scale'))
                    
                coll_geom.append(coll_mesh)
            else:
                # Fallback for primitive shapes (box, cylinder, sphere)
                for primitive in visual_geom:
                    coll_geom.append(ET.fromstring(ET.tostring(primitive)))

            collision.append(coll_geom)

        # Append the completed collision tag to the link
        link.append(collision)
        modified_count += 1
        print(f"Added collision to link: {link.get('name')}")

    # Write the modified tree back to a file
    # 'utf-8' ensures it keeps standard formatting
    tree.write(output_urdf, encoding='utf-8', xml_declaration=True)
    print(f"\nFinished! Modified {modified_count} links. Saved to {output_urdf}")



INPUT_FILE = "/home/pradnesh/Desktop/MMR/Hexapod/hexapod/urdf/hexapod.urdf"
OUTPUT_FILE = "/home/pradnesh/Desktop/MMR/Hexapod/hexapod/urdf/hexapod_with_collisions.urdf"

# If your visual meshes are in 'meshes/visual/' and collisions are in 'meshes/collision/'
# This replaces the string in the <mesh filename="..."> attribute.
# Set to None if your paths inside the URDF should remain completely identical.
FOLDER_MAPPING = ("meshes/", "collision_meshes/")
# ---------------------

add_collision_tags(INPUT_FILE, OUTPUT_FILE, mesh_folder_mapping=FOLDER_MAPPING)