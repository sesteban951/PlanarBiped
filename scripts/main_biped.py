
import mujoco
import numpy as np
import glfw
import time
import os

####################################################################################
# Auxiliary functions
####################################################################################

# Function to compute the center of mass (CoM) of the model
def compute_com(model, data):
    total_mass = 0
    com = np.zeros(3)
    
    # Loop through all bodies in the model
    for i in range(model.nbody):
        # Get the mass of the body
        mass = model.body_mass[i]
        
        # Get the global position of the body's CoM
        xpos = data.xipos[i]
        
        # Accumulate the weighted CoM
        com += mass * xpos
        total_mass += mass
    
    # Divide by the total mass to get the overall CoM
    com /= total_mass
    return com


# Function to update the camera position to track the center of mass (CoM)
def update_camera_to_com(model, data, cam):
    # Calculate the overall center of mass
    com_pos = compute_com(model, data)

    # Set camera parameters to track the CoM
    cam.lookat[:] = com_pos[:3]  # Make the camera look at the CoM
    cam.distance = 3.0  # Distance from the CoM (adjust as needed)
    cam.elevation = -10  # Camera elevation angle (adjust as needed)
    cam.azimuth = 90  # Camera azimuth angle (adjust as needed)


# compute the desired joint positions and velocities
def compute_desired_joint_state(sim_time, data):
    
    # standing configuration
    q_des  = np.array([0.5, -0.2, -0.3, -0.2])  # Initial joint positions
    qd_des = np.array([0.0, 0.0,  0.0, 0.0])   # Initial joint positions

    # rocking out motion
    f = 0.5
    w = 2 * np.pi * f
    q_des[0] +=  0.5 * np.sin(w * sim_time)  # Left hip
    q_des[1] += -0.4 - 0.1 * np.sin(w * sim_time)  # Left hip
    q_des[2] += -0.0 + 0.1* np.sin(w * sim_time)  # Right hip
    q_des[3] += -0.2 + 0.1 * np.sin(w * sim_time)  # Right hip

    return q_des, qd_des


####################################################################################
# Kinematics functions
####################################################################################

# compute the forward kinematics in WORLD Frame
def compute_forward_kinematics(data):

    # lengths of the legs
    l1 = 0.5 # length of the thigh
    l2 = 0.5 # length of the shank

    # unpack the base position
    p_base_W = np.array([data.qpos[0], data.qpos[1]]).reshape(2, 1)
    theta = data.qpos[2]
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])

    # unpack the joint angles
    q_HL = data.qpos[3]
    q_KL = data.qpos[4]
    q_HR = data.qpos[5]
    q_KR = data.qpos[6]

    # compute the positions of the feet relative to the base frame
    p_left_B = np.array([l1 * np.sin(q_HL) + l2 * np.sin(q_HL + q_KL),
                        -l1 * np.cos(q_HL) - l2 * np.cos(q_HL + q_KL)]).reshape(2, 1)
    p_right_B = np.array([l1 * np.sin(q_HR) + l2 * np.sin(q_HR + q_KR),
                         -l1 * np.cos(q_HR) - l2 * np.cos(q_HR + q_KR)]).reshape(2, 1)
    
    # compute the positions of the feet in WORLD frame
    p_left_W = p_base_W + R @ p_left_B
    p_right_W = p_base_W + R @ p_right_B
    
    return p_left_W, p_right_W

# compute inver kineamtics
def compute_inverse_kinematics(p_left, p_right):

    # lengths of the legs
    l1 = 0.5 # length of the thigh
    l2 = 0.5 # length of the shank

    # unpack the desired position
    x_L = p_left[0]
    z_L = p_left[1]
    x_R = p_right[0]
    z_R = p_right[1]

    # compute the angles
    c_L = (x_L**2 + z_L**2 - l1**2 - l2**2) / (2 * l1 * l2)
    s_L = -np.sqrt(1 - c_L**2)
    q_KL = np.arctan2(s_L, c_L)

    c_R = (x_R**2 + z_R**2 - l1**2 - l2**2) / (2 * l1 * l2)
    s_R = -np.sqrt(1 - c_R**2)
    q_KR = np.arctan2(s_R, c_R)

    q_HL = np.arctan2(z_L, x_L) - np.arctan2(l2 * np.sin(q_KL), l1 + l2 * np.cos(q_KL)) + np.pi 
    q_HR = np.arctan2(z_R, x_R) - np.arctan2(l2 * np.sin(q_KR), l1 + l2 * np.cos(q_KR)) + np.pi

    # pack the angles
    q = np.array([q_HL, q_KL, q_HR, q_KR])

    return q

####################################################################################
# Torques functions
####################################################################################

# Function to compute torques based on desired position and velocity
def compute_torques(q_des, qd_des, data):

    # joint gains
    kp_hip = 100
    kd_hip = 10

    kp_knee = 150
    kd_knee = 10

    # unpack the current joint state
    q_left_hip = data.qpos[3]
    q_left_knee = data.qpos[4]
    q_right_hip = data.qpos[5]
    q_right_knee = data.qpos[6]

    qd_left_hip = data.qvel[3]
    qd_left_knee = data.qvel[4]
    qd_right_hip = data.qvel[5]
    qd_right_knee = data.qvel[6]

    # unpack the desired joint state
    q_des_left_hip = q_des[0]
    q_des_left_knee = q_des[1]
    q_des_right_hip = q_des[2]
    q_des_right_knee = q_des[3]

    qd_des_left_hip = qd_des[0]
    qd_des_left_knee = qd_des[1]
    qd_des_right_hip = qd_des[2]
    qd_des_right_knee = qd_des[3]
    
    # compute the torques
    tau_left_hip = kp_hip * (q_des_left_hip - q_left_hip) + kd_hip * (qd_des_left_hip - qd_left_hip)
    tau_left_knee = kp_knee * (q_des_left_knee - q_left_knee) + kd_knee * (qd_des_left_knee - qd_left_knee)
    tau_right_hip = kp_hip * (q_des_right_hip - q_right_hip) + kd_hip * (qd_des_right_hip - qd_right_hip)
    tau_right_knee = kp_knee * (q_des_right_knee - q_right_knee) + kd_knee * (qd_des_right_knee - qd_right_knee)

    # pack the torques
    tau = np.array([tau_left_hip, tau_left_knee, tau_right_hip, tau_right_knee])
    
    return tau
    

####################################################################################
# Simulaiton
####################################################################################

# Simulation loop
def run_simulation():

    # Path to your MuJoCo XML model
    xml_path = "../models/biped/biped.xml"

    # csv data path
    time_file_path = "../data/time.csv"
    pos_file_path = "../data/pos.csv"
    vel_file_path = "../data/vel.csv"
    tau_file_path = "../data/tau.csv"

    # remove the data files if they exist
    try:
        os.remove(time_file_path)
        os.remove(pos_file_path)
        os.remove(vel_file_path)
        os.remove(tau_file_path)
    except OSError:
        pass

    # Load the model
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Set up the GLFW window
    if not glfw.init():
        raise Exception("Could not initialize GLFW")

    window = glfw.create_window(720, 480, "Planar Biped Sim", None, None)
    glfw.make_context_current(window)

    # Create a camera to render the scene
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()

    # Set up scene and context for rendering
    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

    # Set the initial configuration of the robot   
    qpos = np.array([0.0, 1.1,                # position x and z 
                     0.0,                     # theta body
                     0.5, -0.2, -0.3, -0.2])  # left thigh, left knee, right thigh, right knee
    qvel = np.array([0.0, 0.0, 
                     0.0, 
                     0.0, 0.0,  0.0, 0.0])  
    data.qpos[:] = qpos
    data.qvel[:] = qvel

    # Initialize a counter for rendering frames
    step_counter = 0 # Frame counter
    frame_skip = 15  # Only render every so number of steps

    # max sim time
    sim_time = 0.0
    max_sim_time = 150.0

    # Main simulation loop
    while (not glfw.window_should_close(window)) and (sim_time < max_sim_time):

        # get the current mujoco time
        sim_time = data.time

        # compute the desired joint positions and velocities
        q_des, qd_des = compute_desired_joint_state(sim_time, data)

        # compute the torques
        tau = compute_torques(q_des, qd_des, data)

        # Apply the computed torques to the actuators
        data.ctrl[0] = tau[0]
        data.ctrl[1] = tau[1]
        data.ctrl[2] = tau[2]
        data.ctrl[3] = tau[3]

        # compute the forward kinematics
        p_left, p_right = compute_forward_kinematics(data)

        # compute the inverse kinematics
        # q = compute_inverse_kinematics(p_left, p_right)

        print("----------------------------------")
        print("p_left: ", p_left.transpose())
        print("p_right: ", p_right.transpose())
        # print("act: ", data.qpos[3:7])
        # print("ik: ", q)
        # print("(x, z, theta): ", data.qpos[0:3])

        # Update the camera to track the center of mass
        update_camera_to_com(model, data, cam)

        # write the current time and state to csv
        with open(time_file_path, 'a') as f:
            f.write(f"{sim_time}\n")
        with open(pos_file_path, 'a') as f:
            f.write(f"{data.qpos[0]},{data.qpos[1]},{data.qpos[2]},{data.qpos[3]},{data.qpos[4]},{data.qpos[5]},{data.qpos[6]}\n")
        with open(vel_file_path, 'a') as f:
            f.write(f"{data.qvel[0]},{data.qvel[1]},{data.qvel[2]},{data.qvel[3]},{data.qvel[4]},{data.qvel[5]},{data.qvel[6]}\n")
        with open(tau_file_path, 'a') as f:
            f.write(f"{tau[0]},{tau[1]},{tau[2]},{tau[3]}\n")
        
        # Step the simulation
        mujoco.mj_step(model, data)

        # Increment the step counter
        step_counter += 1
        if step_counter % frame_skip == 0:
            # Get framebuffer size and create viewport for rendering
            width, height = glfw.get_framebuffer_size(window)
            viewport = mujoco.MjrRect(0, 0, width, height)

            # Update scene for rendering
            mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
            mujoco.mjr_render(viewport, scene, context)

            # Swap buffers and poll events for window
            glfw.swap_buffers(window)

        # Poll for window events like keypress or close
        glfw.poll_events()


####################################################################################
# Main execution
####################################################################################

try:
    # main simulation loop
    run_simulation()
finally:
    # Cleanup
    glfw.terminate()
