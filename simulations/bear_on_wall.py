from bs4 import BeautifulSoup
import mujoco
import mujoco.viewer
import argparse
import logging
import pathlib
import numpy as np
import glfw
import time

def merge_mujoco_soups(soup1, soup2):
    # Merge assets
    assets1 = soup1.find('asset')
    assets2 = soup2.find('asset')
    if assets1 and assets2:
        for item in assets2.find_all(recursive=False):
            assets1.append(item)

    # Merge worldbody
    worldbody1 = soup1.find('worldbody')
    worldbody2 = soup2.find('worldbody')
    if worldbody1 and worldbody2:
        for body in worldbody2.find_all('body', recursive=False):
            worldbody1.append(body)

    # Merge default
    default1 = soup1.find('default')
    default2 = soup2.find('default')
    if default1 is None and default2 is not None:
        default1 = soup1.new_tag('default')
        soup1.mujoco.append(default1)

    if default1 and default2:
        for c in default2.find_all(recursive=False):
            default1.append(c)

    actuators1 = soup1.find('actuator')
    actuators2 = soup2.find('actuator')
    if actuators1 is None and actuators2 is not None:
        actuators1 = soup1.new_tag('actuator')
        soup1.mujoco.append(actuators1)

    if actuators1 and actuators2:
        for actuator in actuators2.find_all(recursive=False):
            actuators1.append(actuator)
    
    return soup1

def fix_assets(env, assets_dir):
    if assets_dir:
        meshes = env.find_all('mesh', file=True)
        for mesh in meshes:
            mesh['file'] = str(assets_dir + '/' + mesh['file'])

        textures = env.find_all('texture', file=True)
        for tex in textures:
            tex['file'] = str(assets_dir + '/' + tex['file'])

    return env

def create_bear_walking_simulation(model_path, assets_dir = None, initial=None):
    if assets_dir is None:
        assets_dir = str(pathlib.Path(model_path).parent)
    
    bear = load_model(model_path, assets_dir)

    ground_plane_xml = """
<mujoco model="ground plane">
    <compiler angle="radian" autolimits="true"/>
    <asset>
        <!-- Define checker texture -->
        <texture name="checker_tex" type="2d" builtin="checker" width="512" height="512" rgb1="0.2 0.2 0.6" rgb2="0.8 0.8 1.0"/>
        <material name="checker_mat" texture="checker_tex" texrepeat="3 5" />
    </asset>

    <worldbody>
        <body name="ground" pos="0 0 0">
            <geom name="floor" size="10 10 0.01" type="plane" material="checker_mat" rgba="1 1 1 1"/>
        </body>
        <body name="Head Target" pos="0.8 -0.05 0.40" quat="1 0 0 0" mocap="true">
            <site type="sphere" size="0.01" rgba="0 0 1 1" group="1"/>
        </body>
    </worldbody>
</mujoco>
"""
    # env = BeautifulSoup(ground_plane_xml, features='xml')
    
    with open("../mujoco/groundplane.xml", encoding='utf-8') as f:
        xml_s = f.read()
        env = BeautifulSoup(xml_s, features='xml')

    with open("../mujoco/wall.xml", encoding='utf-8') as f:
        xml_s = f.read()
        xml_obj = BeautifulSoup(xml_s, features='xml')
        env = merge_mujoco_soups(env, xml_obj)
    
    with open("../mujoco/route_V0_2.xml", encoding='utf-8') as f:
        xml_s = f.read()
        xml_obj = BeautifulSoup(xml_s, features='xml')
        env = merge_mujoco_soups(env, xml_obj)

    with open("../mujoco/taiwanbear.xml", encoding='utf-8') as f:
        xml_s = f.read()
        xml_obj = BeautifulSoup(xml_s, features='xml')
        env = merge_mujoco_soups(env, xml_obj)

    cam_xml = f""" 
        <camera pos="1.377 -4.506 2.566" xyaxes="0.956 0.292 0.000 -0.140 0.457 0.878"/>
    """
    cam_bs = BeautifulSoup(cam_xml, features='xml')
    env.mujoco.worldbody.append(cam_bs)

    acts = env.find('actuator')
    if acts:
        for act in acts:
            act['kp'] = 90.0
            act['forcerange'] = '-150 150'

    env = fix_assets(env, assets_dir)

     # Save the merged model XML for inspection
    with open("merged_model.xml", 'w') as file:
        file.write(env.prettify())

     # Load the modified model into MuJoCo

    model = mujoco.MjModel.from_xml_string(str(env))
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)

    return model, data

def target_trajectory(t: float, r: float, h: float, j: float, k: float, f: float) -> np.ndarray:
    """Return the (x, y) coordinates of a circle with radius r centered at (h, k)
    as a function of time t and frequency f."""
    x = h + r * np.sin(np.pi * f * t)
    y = j + r * np.cos(np.pi * f * t)
    z = k + r * np.sin(np.pi * f * t)

    return np.array([x, y, z, 1])    

def add_visual_capsule(scene, point1, point2, radius, rgba):
    """Adds one capsule to an mjvScene."""
    if scene.ngeom >= scene.maxgeom:
        return
    scene.ngeom += 1  # increment ngeom
    # initialise a new capsule, add it to the scene using mjv_makeConnector
    mujoco.mjv_initGeom(scene.geoms[scene.ngeom-1],
                        mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                        np.zeros(3), np.zeros(9), rgba.astype(np.float32))
    # mujoco.mjv_makeConnector(scene.geoms[scene.ngeom-1],
    #                         mujoco.mjtGeom.mjGEOM_CAPSULE, radius,
    #                         point1[0], point1[1], point1[2],
    #                         point2[0], point2[1], point2[2])
  
def modify_scene(scn, target_traj, end_effector_traj):
    """Draw position trace"""
    if len(target_traj) > 1:
        for i in range(len(target_traj)-1):
            add_visual_capsule(scn, target_traj[i], target_traj[i+1], 0.005, np.array([0, 0, 1.0, 1.0]))
            add_visual_capsule(scn, end_effector_traj[i], end_effector_traj[i+1], 0.005, np.array([1.0, 0, 0, 0.8]))

def apply_ik_step(model, data, site_id, mocap_id):

    # Pre-allocate numpy arrays.
    jac = np.zeros((6, model.nv))
    error = np.zeros(6)
    error_pos = error[:3]
    error_ori = error[3:]
    site_quat = np.zeros(4)
    target_quat_conj = np.zeros(4)
    error_quat = np.zeros(4)

    """Perform one IK step to move the end-effector towards the target position."""
    current_pos = data.site(site_id).xpos
    error_pos[:] = data.site(site_id).xpos - data.mocap_pos[mocap_id]

    target_ori = data.mocap_quat[mocap_id]

    mujoco.mju_negQuat(target_quat_conj, target_ori)#
    mujoco.mju_mat2Quat(site_quat, data.site(site_id).xmat)
    mujoco.mju_mulQuat(error_quat, site_quat, target_quat_conj)
    # Convert error quaternion to axis-angle representation
    # We do so, as the Jacobian function we will use represent orientation error in axis-angle form
    mujoco.mju_quat2Vel(error_ori, error_quat, 1.0)
    
    # Get the Jacobian with respect to the end-effector site.
    # This function calculate the Jacobian of the world coordinates of a body frame
    mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id)
    #print("JAC", jac.shape, jac)
    
    # Solve the differential IK
    # We want to have the error equal to zero
    # We take a step dq such J dq = -error 
    # Note, the origin differential IK works on J v = -speed * error / dt
    # Here we implement a simple version by using dq and making Jdq = -error.
    dq = np.linalg.pinv(jac) @ -error

    #print(dq)

    # Our robot arm is position controlled, so we simple give it the target joint configure
    q = data.qpos.copy()
    # Add dq to q, here results should be the same as q = q + dq. It is different when q includes quaternian
    mujoco.mj_integratePos(model, q, dq, 1)
    
    control = np.zeros((model.nv + 1,3))
    control[:,0] = data.qpos
    control[:,1] = q
    control[:,2] = control[:,1] - control[:,0]

    #print("Control", control)

    # Our robot is configured to be position control
    # Here we direct set the control signal to the desired position

    #print("T", model.nv, q.shape, *model.jnt_range.T.shape)
    #print("JN", model.jnt_range)
    joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.ngeom)]
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.ngeom)]

    # print(model.ngeom, model.nv, model.nu, len(q))
    # print(len(joint_names),joint_names)
    # print(len(actuator_names), actuator_names)

    x = data.ctrl.copy() # np.zeros(model.nu) # q[6:].copy()
    #x[-1] = x[-1] - dq[-1]  
    #print("Joint ranges", model.jnt_range)
    #np.clip(x, *model.jnt_range.T, out=x)
    #data.ctrl = q[1:2]
    print("c", error[0:3], control[-3:,:], data.ctrl[-3:])
    data.ctrl[-1] = data.qpos[-1]

def calc_mapping(model, data):
    joint_mapping = {}
    for i in range(model.nv):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name:
            joint_qpos_adr = model.jnt_qposadr[i]
            joint_qvel_adr = model.jnt_dofadr[i]
            joint_mapping[joint_name] = (joint_qpos_adr, joint_qvel_adr)

    actuator_mapping = {}
    for i in range(model.nv):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if act_name:
            
            actuator_mapping[joint_name] = (joint_qpos_adr, joint_qvel_adr)

    print("Joint mapping:", joint_mapping)
    print("Actuator mapping:", actuator_mapping)

    return joint_mapping, actuator_mapping

def run_simulation(model_path, assets_dir = None, initial=None):
    model, data = create_bear_walking_simulation(model_path, assets_dir)  
    #mocap_id = model.body("Head Target").mocapid[0]
    #site_id = model.site("Head Site").id

    with mujoco.viewer.launch_passive(model, data) as viewer:    
        while viewer.is_running():
            mujoco.mj_step(model, data)

            viewer.sync()
            #time.sleep(0.001)
            
def run_simulation_glfw(model_path, assets_dir = None, initial=None):
    model, data = create_bear_walking_simulation(model_path, assets_dir)  
    mocap_id = model.body("Head Target").mocapid[0]
    site_id = model.site("Head Site").id

    target_traj = []
    end_effector_traj = []

    glfw.init()
    window = glfw.create_window(640, 480, "MuJuCo Window", None, None)
    glfw.make_context_current(window)

    scene = mujoco.MjvScene(model, maxgeom=1000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultCamera(cam)

    data.mocap_pos[mocap_id, 0:2] = target_trajectory(data.time, r=0.6, h=0.0, k=0.0, f=0.05)
        
    while not glfw.window_should_close(window):
        mujoco.mj_step(model, data)

        target_traj.append(data.mocap_pos[mocap_id].copy())
        end_effector_traj.append(data.site(site_id).xpos.copy())
        modify_scene(scene, target_traj, end_effector_traj)

        # Update scene
        mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)

        # Get framebuffer viewport
        width, height = glfw.get_framebuffer_size(window)

        # Render
        viewport = mujoco.MjrRect(0, 0, 640, 480)
        mujoco.mjr_render(viewport, scene, context)

        # Swap OpenGL buffers
        glfw.swap_buffers(window)
        glfw.poll_events()

    glfw.terminate()

def load_model(model_path, assets_dir = None, initial=None):
    # Load and parse the MuJoCo model XML file
    
    with open(model_path, 'r') as file:
        model_xml = file.read()
    soup = BeautifulSoup(model_xml, 'xml')
    
    if assets_dir:
        meshes = soup.find_all('mesh')
        for mesh in meshes:
            mesh['file'] = str(assets_dir + '/' + mesh['file'])

    # Set initial attributes if provided
    bear_el = soup.find("body", {"name": "TaiwanBear"})
    #bear_el = soup.find("body", {"name": "root"})
    if bear_el:
        bear_el['pos'] = "0 0 0.42"
        bear_el['euler'] = "0 0 0"
    else:
        raise ValueError("Could not find TaiwanBear body in the model XML.")
            
    # Modify the model to add walking capabilities if necessary
    # (This is a placeholder for any modifications needed)
    
    # Convert back to string
    modified_model_xml = str(soup)
    
    # Load the modified model into MuJoCo
    #model = mujoco.MjModel.from_xml_string(modified_model_xml)
    #data = mujoco.MjData(model)
    
    return soup

def main(argv = None):
    if argv is None:
        import sys
        argv = sys.argv[1:]
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, default="model.xml", help="Path to the MuJoCo model XML file.")
    parser.add_argument("--assets", type=str, default=None, help="Path to the assets directory from the MJCF model file.")
    
    parser.add_argument("--initial", type=str, default=None, help="Initial attributes of the model.")
    
    parser.add_argument("--verbsoe", "-v", action="count", default=0, help="Increase verbosity level.")
    
    args = parser.parse_args(argv)
    run_simulation(args.model, args.assets, args.initial)
    
if __name__ == "__main__":
    #main(["--model", "./mujoco/op3.xml"])
    main(["--model", "../mujoco/taiwanbear.xml", '--assets', '../mujoco'])

