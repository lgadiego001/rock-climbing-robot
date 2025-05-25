import bpy, math
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False, confirm=False)

bpy.ops.mesh.primitive_cylinder_add(enter_editmode=False, align='WORLD', location=(0, 0, 0.5), scale=(1.5, 1.5, 0.5))
bpy.context.active_object.name = "base"
bpy.ops.mesh.primitive_uv_sphere_add(radius=1, enter_editmode=False, align='WORLD', location=(0, 0, 1), scale=(1, 1, 1))
bpy.context.active_object.name = "joint_1_XYZ"
bpy.ops.mesh.primitive_cylinder_add(radius=1, depth=2, enter_editmode=False, align='WORLD', location=(0, 0, 4), scale=(0.5, 0.5, 3))
bpy.context.active_object.name = "link1"
bpy.ops.mesh.primitive_uv_sphere_add(radius=1, enter_editmode=False, align='WORLD', location=(0, 0, 7), scale=(1, 1, 1))
bpy.context.active_object.name = "joint_2_XYZ"
bpy.ops.mesh.primitive_cylinder_add(radius=1, depth=2, enter_editmode=False, align='WORLD', location=(0, 0, 10), scale=(0.5, 0.5, 3), rotation=(0,0,0))
bpy.context.active_object.name = "link2"
bpy.ops.mesh.primitive_cylinder_add(radius=1, depth=2, enter_editmode=False, align='WORLD', location=(0, 0, 13), scale=(1, 1, 0.8), rotation=(math.radians(90),0,0))
bpy.context.active_object.name = "joint_3_Z"
bpy.ops.transform.rotate(value=-1.5708, orient_axis='X', orient_type='LOCAL', orient_matrix=((1, 0, 0), (0, -4.37114e-08, 1), (0, -1, -4.37114e-08)), orient_matrix_type='LOCAL', constraint_axis=(True, False, False), mirror=False, use_proportional_edit=True, proportional_edit_falloff='SMOOTH', proportional_size=1.21, use_proportional_connected=False, use_proportional_projected=False)
bpy.ops.mesh.primitive_cylinder_add(radius=1, depth=2, enter_editmode=False, align='WORLD', location=(0, 0, 16), scale=(0.5, 0.5, 3), rotation=(0,0,0))
bpy.context.active_object.name = "link3"
bpy.ops.mesh.primitive_uv_sphere_add(radius=1, enter_editmode=False, align='WORLD', location=(0, 0, 19), scale=(1, 1, 1))
bpy.context.active_object.name = "end_effector"

# parenting
objects = bpy.data.objects
l = [objects["base"], objects["joint_1_XYZ"], objects["link1"], objects["joint_2_XYZ"], objects["link2"], objects["joint_3_Z"], objects["link3"], objects["end_effector"]]
iter = 1
for idx in range(len(l)-1):
    bpy.ops.object.select_all(action='DESELECT')
    l[iter].select_set(True)
    l[iter-1].select_set(True)
    bpy.context.view_layer.objects.active = l[iter-1]
    bpy.ops.object.parent_set(type='OBJECT', xmirror=False, keep_transform=False)
    iter += 1

bpy.ops.mesh.primitive_ico_sphere_add(radius=1, enter_editmode=False, align='WORLD', location=(2, 7, 5), scale=(1, 1, 1))
bpy.context.active_object.name = "Point_1"
bpy.ops.mesh.primitive_ico_sphere_add(radius=1, enter_editmode=False, align='WORLD', location=(-3, -5, 7), scale=(1, 1, 1))
bpy.context.active_object.name = "Point_2"
bpy.ops.mesh.primitive_ico_sphere_add(radius=1, enter_editmode=False, align='WORLD', location=(4, -5, 1), scale=(1, 1, 1))
bpy.context.active_object.name = "Point_3"

def ik(x,y,z):
    # write your code here
    pass
    

point = 3

match(point):
    case 1:
        ik(objects["Point_1"].location[0],objects["Point_1"].location[1],objects["Point_1"].location[2])
    case 2:
        ik(objects["Point_2"].location[0],objects["Point_2"].location[1],objects["Point_2"].location[2])
    case 3:
        ik(objects["Point_3"].location[0],objects["Point_3"].location[1],objects["Point_3"].location[2])
    case _:
        pass