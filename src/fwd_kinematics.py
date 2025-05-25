import bpy
from mathutils import Matrix
import pprint

def get_transformation_matrix( obj_a, obj_b):
    relative_transform_matrix = Matrix.identity
    if ((obj_a is not None) and (obj_b is not None)):
        matrix_a = obj_a.matrix_world
        matrix_b = obj_b.matrix_world
        # Calculate the relative transformation matrix
        relative_transform_matrix = matrix_a.inverted() @ matrix_b
    return relative_transform_matrix


def decompose_transformation_matrix(m):
    # Extract relative position, rotation, and scale
    r_position = m.to_translation()
    r_rotation = m.to_euler()  # Or use .to_quaternion() for quaternion
    r_scale = m.to_scale()
    return r_position, r_rotation, r_scale

def get_parent_joint(obj):
    parent = obj.parent
    #print('get_parent',parent.name)
    while((parent.parent is not None) and (not parent.name.startswith("RJoint_"))):
        parent = parent.parent
    return parent
        
def get_forward_kinematics(root_obj):
    if ((root_obj.parent is not None) and ((root_obj.name.startswith("RJoint")) or (root_obj.name.startswith("Effector")))):
        root_par = get_parent_joint(root_obj)
        root_m = get_transformation_matrix(root_par, root_obj)
    else:
        root_par = None
        root_m = None
    print('fwd', root_obj.name, root_par, root_m)
    children = []
    for child in root_obj.children:
        fwd = get_forward_kinematics(child)
        children.append(fwd)
    return (root_obj, root_par, root_m, children)

def print_forward_kinematics(fwd, level = 0):
    curr_obj, curr_parent, curr_matrix, curr_children = fwd
    if (curr_matrix is not None):
        pos, rot, scale = decompose_transformation_matrix(curr_matrix)
        print("   " * level, curr_obj.name, curr_parent.name, pos, len(curr_children))
    for c in curr_children:
        print_forward_kinematics(c, level + 1)
    
    
# Replace 'ObjectA' and 'ObjectB' with the names of your objects
obj_body = bpy.data.objects['Body']
obj_b = bpy.data.objects['RJoint_Back_Upper_XYZ_L']

t = get_transformation_matrix(obj_body, obj_b)
print("Transformation matrix", t)

r_position, r_rotation, r_scale = decompose_transformation_matrix(t)

print("Relative Position:", r_position)
print("Relative Rotation (Euler):", r_rotation)
print("Relative Scale:", r_scale)

obj_a = bpy.data.objects['Body']
obj_b = bpy.data.objects['Joint_Back_Upper_XYZ_R']

t = get_transformation_matrix(obj_body, obj_b)
print("Transformation matrix", t)

r_position, r_rotation, r_scale = decompose_transformation_matrix(t)

print("Relative Position:", r_position)
print("Relative Rotation (Euler):", r_rotation)
print("Relative Scale:", r_scale)

fwd = get_forward_kinematics(obj_body)
#pprint.pprint(fwd)
print_forward_kinematics(fwd)
