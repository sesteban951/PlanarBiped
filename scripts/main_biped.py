
# standard library
import numpy as np
import scipy as sp
import os
import copy

# mujoco stuff
import mujoco
import glfw

# bezier stuff
import bezier

# yaml stuff
import yaml

####################################################################################
# Biped Simulation
####################################################################################

class BipedSimulation:

    def __init__(self, config):
        
        # load the model
        xml_path = "../models/biped/biped.xml"
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # initial state
        qpos = np.array([config["INIT"]["q0"]])
        qvel = np.array([config["INIT"]["v0"]])
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel

        # geomtry id (in case I want to query the position)
        self.left_foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "foot_left")
        self.right_foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "foot_right")
        
        # sim parameters
        self.sim_time = 0.0
        self.max_sim_time = config["SIM"]["sim_time"]
        self.dt_sim = self.model.opt.timestep
        self.hz_render = config["SIM"]["hz_render"]

        # model parameters
        self.l1 = 0.5 # length of the thigh (check that this matches the XML file)
        self.l2 = 0.5 # length of the shin  (check that this matches the XML file)

        # center of mass state
        self.p_com = None
        self.v_com = None

        # ROM state
        self.p_rom = None
        self.v_rom = None

        # desired COM state
        self.v_des = config["HLIP"]["v_des"]

        # phasing variables
        self.T_SSP = config["HLIP"]["T_SSP"]
        self.T_DSP = 0.0
        self.T_tot = self.T_SSP + self.T_DSP
        self.T_phase = 0.0
        self.num_steps = 0
        self.stance_foot = None

        # foot position variables
        self.p_swing_init = np.zeros(2).reshape(2,1) # previous stance foot position
        self.p_swing = np.zeros(2).reshape(2,1)      # current swing foot position
        self.p_stance = np.zeros(2).reshape(2,1)     # current stance foot position
        self.u = None                                # foot placement w.r.t to stance foot

        # desired height
        self.theta_des = config["HLIP"]["theta_des"]   # desired torso angle
        self.z_0 = config["HLIP"]["z0"]                      # LIP constant height
        self.z_apex = config["HLIP"]["z_apex"]               # foot apex height
        self.z_foot_offset = config["HLIP"]["z_foot_offset"] # (foot is round)
        self.bez_deg = config["HLIP"]["bezier_deg"]               # degree of bezier curve

        # some hyperbolic trig lambda func
        self.coth = lambda x: (np.exp(2 * x) + 1) / (np.exp(2 * x) - 1)

        # HLIP matrices
        g = -self.model.opt.gravity[2]
        self.lam = np.sqrt(g / self.z_0)
        self.A = np.array([[0,           1],
                           [self.lam**2, 0]])
        self.Kp_db = 1
        self.Kd_db = self.T_DSP + (1/self.lam) * self.coth(self.lam * self.T_SSP)
        self.sigma_P1 = self.lam * self.coth(0.5 * self.lam * self.T_SSP)
        self.u_bias = config["HLIP"]["u_bias"]

        # low level joint gains
        self.kp_H = config["GAINS"]["kp_H"]
        self.kd_H = config["GAINS"]["kd_H"]
        self.kp_K = config["GAINS"]["kp_K"]
        self.kd_K = config["GAINS"]["kd_K"]

        # camera settings
        self.cam_distance = config["CAMERA"]["distance"]
        self.cam_elevation = config["CAMERA"]["elevation"]
        self.cam_azimuth = config["CAMERA"]["azimuth"]
        self.pz_com_offset = config["CAMERA"]["pz_com_offset"]

    ############################################### ROM ######################################

    # update the phasing variable
    def update_phase(self):

        # compute the number of steps taken so far
        self.num_steps = int(self.sim_time / self.T_SSP)

        # update the phase variable
        self.T_phase += self.dt_sim 
        self.T_phase = np.clip(self.T_phase, 0.0, self.T_SSP)

        # update the stance foot
        if self.num_steps % 2 == 0:
            self.stance_foot = "L"
        else:
            self.stance_foot = "R"

    # update which foot is swing and stance
    def update_stance_foot(self):

        # compute the forward kinematics
        _, y_left_W, y_right_W = self.compute_forward_kinematics()

        # based on the phase variable, assign stance foot position
        if self.stance_foot == "L":
            self.p_swing_init = np.array([y_right_W[0], [self.z_foot_offset]]).reshape(2, 1)
            self.p_stance = np.array([y_left_W[0], [self.z_foot_offset]]).reshape(2, 1)

        elif self.stance_foot == "R":
            self.p_swing_init = np.array([y_left_W[0], [self.z_foot_offset]]).reshape(2, 1)
            self.p_stance = np.array([y_right_W[0], [self.z_foot_offset]]).reshape(2, 1)

    # compute center of mass (COM) state
    def update_com_state(self):

        # compute COM position
        total_mass = 0
        com = np.zeros(3)
        
        # Loop through all bodies in the model
        for i in range(self.model.nbody):
            # Get the mass of the body
            mass = self.model.body_mass[i]
            
            # Get the global position of the body's CoM
            xpos = self.data.xipos[i]
            
            # Accumulate the weighted CoM
            com += mass * xpos
            total_mass += mass
        
        # Divide by the total mass to get the overall CoM
        com /= total_mass

        # update the COM state
        self.p_com = np.array([com[0], com[2]]).reshape(2, 1)
        self.v_com = np.array([self.data.qvel[0], self.data.qvel[1]]).reshape(2, 1)

    # compute hip lateral position (HLIP) state
    def update_hlip_state(self):

        # compute the forward kinematics
        _, y_left_W, y_right_W = self.compute_forward_kinematics()

        # compute the HLIP position
        if self.stance_foot == "L":
            p = self.p_com[0] - y_left_W[0]
        elif self.stance_foot == "R":
            p = self.p_com[0] - y_right_W[0]

        self.p_rom = p[0]
        self.v_rom = self.v_com[0][0]

    # compute the foot placement based on Raibert
    def compute_foot_placement(self):

        # compute the desired preimpact HLIP state
        p_minus_H = (self.v_des * self.T_tot) / (2 + self.T_DSP * self.sigma_P1)
        v_minus_H = self.sigma_P1 * (self.v_des * self.T_tot) / (2 + self.T_DSP * self.sigma_P1)

        # compute the predicted preimpact state of the robot
        x_R = np.array([self.p_rom, self.v_rom]).reshape(2, 1)
        x_minus_R = sp.linalg.expm(self.A * (self.T_SSP - self.T_phase)) @ x_R
        p_minus_R = x_minus_R[0][0]
        v_minus_R = x_minus_R[1][0]

        # compute the foot placement relative to the stance foot
        u_ff = self.v_des * self.T_SSP
        u_pos = self.Kp_db * (p_minus_R - p_minus_H)
        u_vel = self.Kd_db * (v_minus_R - v_minus_H)

        self.u = u_ff + u_pos + u_vel + self.u_bias

    ############################################### KINEMATICS ######################################

    # compute swing foot based on foot placement and bezier curve
    def compute_swing_foot_pos(self):

        # compute the foot placement
        self.compute_foot_placement()

        # initial and end points
        x0 = self.p_swing_init[0][0]
        xf = self.p_stance[0][0] + self.u
        xm =(x0 + xf) * 0.5

        z0 = self.z_foot_offset
        zf = self.z_foot_offset

        # bezier curve points
        if self.bez_deg == 5:
            zm = (self.z_apex + self.z_foot_offset) * (8/3)
            x_pts = np.array([x0, x0, x0, xm, xf, xf, xf]) # x Bezier points
            z_pts = np.array([z0, z0, z0, zm, zf, zf, zf]) # z Bezier points
        elif self.bez_deg == 7:
            zm = (self.z_apex + self.z_foot_offset) * (16/5) 
            x_pts = np.array([x0, x0, xm, xf,  xf]) # x Bezier points
            z_pts = np.array([z0, z0, zm, zf,  zf]) # z Bezier points
        
        # bezier control points
        P = np.array([x_pts, z_pts])

        # want to compute this at normalized time
        t = self.T_phase / self.T_SSP

        # clip t to be between 0 and 1
        t = np.clip(t, 0.0, 1.0)

        # build the bezier curve
        nodes = np.asfortranarray(P)
        curve = bezier.Curve(nodes, degree=(P.shape[1]-1))
        
        # evaulate the bezier curve at time t
        p_swing_bez = curve.evaluate(t)

        # get the current swing foot position
        _, y_left_W, y_right_W = self.compute_forward_kinematics()
        if self.stance_foot == "L":
            p_swing_act = y_right_W
        elif self.stance_foot == "R":
            p_swing_act = y_left_W
        
        # convex combination of bezier and current foot position
        # p_swing = (1 - t) * p_swing_act + t * p_swing_bez
        p_swing = p_swing_bez

        return p_swing

    # update the desired outputs
    def update_output_des(self):
        
        # compute the desired base outputs
        y_base_W, _, _ = self.compute_forward_kinematics()
        y_base_W[1] = self.z_0 + self.z_foot_offset
        y_base_W[2] = self.theta_des
        y_base_des_W = y_base_W
        
        # define the desired foot outputs
        p_swing_W = self.compute_swing_foot_pos()

        # make a deep copy of the stance foot
        p_stance_W = self.p_stance

        # compute the desired foot outputs
        if self.stance_foot == "L":
            y_left_des_W = p_stance_W
            y_right_des_W = p_swing_W

        elif self.stance_foot == "R":
            y_left_des_W = p_swing_W
            y_right_des_W = p_stance_W

        return y_base_des_W, y_left_des_W, y_right_des_W

    # compute the forward kinematics in WORLD Frame
    def compute_forward_kinematics(self):

        # unpack the base position
        p_base_W = np.array([self.data.qpos[0], self.data.qpos[1]]).reshape(2, 1)
        theta_W = self.data.qpos[2]
        R = np.array([[np.cos(theta_W), -np.sin(theta_W)],
                      [np.sin(theta_W),  np.cos(theta_W)]])
        y_base_W = np.array([p_base_W[0], p_base_W[1], [theta_W]]).reshape(3, 1)

        # unpack the joint angles
        q_HL = self.data.qpos[3]
        q_KL = self.data.qpos[4]
        q_HR = self.data.qpos[5]
        q_KR = self.data.qpos[6]

        # compute the positions of the feet relative to the base frame
        p_left_B = np.array([self.l1 * np.sin(q_HL) + self.l2 * np.sin(q_HL + q_KL),
                            -self.l1 * np.cos(q_HL) - self.l2 * np.cos(q_HL + q_KL)]).reshape(2, 1)
        p_right_B = np.array([self.l1 * np.sin(q_HR) + self.l2 * np.sin(q_HR + q_KR),
                             -self.l1 * np.cos(q_HR) - self.l2 * np.cos(q_HR + q_KR)]).reshape(2, 1)
        p_left_W = p_base_W + R @ p_left_B
        p_right_W = p_base_W + R @ p_right_B
        y_left_W = p_left_W
        y_right_W = p_right_W

        return y_base_W, y_left_W, y_right_W

    # compute inverse kineamtics given feet position in world frame
    def compute_inverse_kinematics(self, y_base_W, y_left_W, y_right_W):

        # unpack the base position
        p_base_W = np.array([y_base_W[0], y_base_W[1]]).reshape(2, 1)
        theta_des = y_base_W[2]
        R = np.array([[np.cos(theta_des), -np.sin(theta_des)],
                      [np.sin(theta_des),  np.cos(theta_des)]])

        # compute the position of the feet in base frame
        p_left_W = np.array([y_left_W[0], y_left_W[1]]).reshape(2, 1)
        p_right_W = np.array([y_right_W[0], y_right_W[1]]).reshape(2, 1)
        p_left_B = (R.transpose() @ (p_left_W - p_base_W)).reshape(2, 1)
        p_right_B = (R.transpose() @ (p_right_W - p_base_W)).reshape(2, 1)

        # unpack the desired position feet positions in world frame
        x_L = p_left_B[0][0]
        z_L = p_left_B[1][0]
        x_R = p_right_B[0][0]
        z_R = p_right_B[1][0]

        # knee angle (Left)
        L = np.sqrt(x_L**2 + z_L**2)
        arg = np.clip((L**2 - self.l1**2 - self.l2**2) / (-2 * self.l1 * self.l2),  -1.0, 1.0)
        gamma = np.arccos(arg)
        q_KL = gamma - np.pi

        # hip angle (Left)
        beta = np.arctan2(x_L, -z_L)
        arg = np.clip((self.l2**2 - self.l1**2 - L**2) / (-2 * self.l1 * L),  -1.0, 1.0)
        alpha = np.arccos(arg)
        q_HL = beta + alpha

        # knee angle (Right)
        L = np.sqrt(x_R**2 + z_R**2)
        arg = np.clip((L**2 - self.l1**2 - self.l2**2) / (-2 * self.l1 * self.l2),  -1.0, 1.0)
        gamma = np.arccos(arg)
        q_KR = gamma - np.pi
        
        # hip angle (Right)
        beta = np.arctan2(x_R, -z_R)
        arg = np.clip((self.l2**2 - self.l1**2 - L**2) / (-2 * self.l1 * L),  -1.0, 1.0)
        alpha = np.arccos(arg)
        q_HR = beta + alpha

        # pack the positions
        q_left = np.array([q_HL, q_KL]).reshape(2, 1)
        q_right = np.array([q_HR, q_KR]).reshape(2, 1)

        return q_left, q_right
    
    ############################################### LOW LEVEL ######################################

    # Function to compute torques based on desired position and velocity
    def compute_torques(self, q_des, qd_des):

        # unpack the current joint state
        q_HL = self.data.qpos[3]
        q_KL = self.data.qpos[4]
        q_HR = self.data.qpos[5]
        q_KR = self.data.qpos[6]

        qd_HL = self.data.qvel[3]
        qd_KL = self.data.qvel[4]
        qd_HR = self.data.qvel[5]
        qd_KR = self.data.qvel[6]

        # unpack the desired joint state
        q_HL_des = q_des[0]
        q_KL_des = q_des[1]
        q_HR_des = q_des[2]
        q_KR_des = q_des[3]

        qd_HL_des = qd_des[0]
        qd_KL_des = qd_des[1]
        qd_HR_des = qd_des[2]
        qd_KR_des = qd_des[3]
        
        # compute the torques
        tau_HL = self.kp_H * (q_HL_des - q_HL) + self.kd_H * (qd_HL_des - qd_HL)
        tau_KL = self.kp_H * (q_KL_des - q_KL) + self.kd_K * (qd_KL_des - qd_KL)
        tau_HR = self.kp_H * (q_HR_des - q_HR) + self.kd_H * (qd_HR_des - qd_HR)
        tau_KR = self.kp_H * (q_KR_des - q_KR) + self.kd_K * (qd_KR_des - qd_KR)

        # pack the torques
        tau = [tau_HL, tau_KL, tau_HR, tau_KR]
        
        return tau
    
    ############################################### AUX FUNC ######################################

    # to see where the COM is in the simulation
    def update_com_visualization(self):
        p_com_flat = np.ravel(self.p_com)
        com_pos = np.array([p_com_flat[0], 0.0, p_com_flat[1]])
        com_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "com_vis")
        if com_geom_id != -1:
            self.model.geom_pos[com_geom_id] = com_pos  # Update position

    # to see where the foot targets are at
    def update_foot_visualization(self, y_left_W, y_right_W):

        p_left_flat = np.ravel(y_left_W)
        p_right_flat = np.ravel(y_right_W)
        left_pos = np.array([p_left_flat[0], 0.10, p_left_flat[1]])
        right_pos = np.array([p_right_flat[0], -0.10, p_right_flat[1]])
        left_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "left_vis")
        right_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "right_vis")
        if left_geom_id != -1:
            self.model.geom_pos[left_geom_id] = left_pos  # Update position
        if right_geom_id != -1:
            self.model.geom_pos[right_geom_id] = right_pos  # Update position

    # Function to update the camera position to track the center of mass (CoM)
    def update_camera_to_com(self, cam):

        # Set camera parameters to track the CoM
        cam.lookat[:] = [self.p_com[0][0], 0, self.p_com[1][0] + self.pz_com_offset]  # Make the camera look at the CoM

    ############################################### SIMULATION ######################################

    # run the simulation
    def simulation(self):
        
        # csv data path
        time_file_path = "../data/time.csv"
        
        # whole body state
        pos_file_path = "../data/pos.csv"
        vel_file_path = "../data/vel.csv"
        tau_file_path = "../data/tau.csv"

        # joint state
        q_des_path = "../data/q_des.csv"
        q_act_path = "../data/q_act.csv"

        # desired outputs
        y_des_path = "../data/y_des.csv"
        y_act_path = "../data/y_act.csv"

        # phase
        t_phase_path = "../data/t_phase.csv"
        stance_path = "../data/stance.csv"

        # rom state
        rom_state_path = "../data/rom_state.csv"
        rom_input_path = "../data/rom_input.csv"

        # remove the data files if they exist
        try:
            os.remove(time_file_path)
            os.remove(pos_file_path)
            os.remove(vel_file_path)
            os.remove(tau_file_path)
            os.remove(q_des_path)
            os.remove(q_act_path)
            os.remove(y_des_path)
            os.remove(y_act_path)
            os.remove(t_phase_path)
            os.remove(stance_path)
            os.remove(rom_state_path)
            os.remove(rom_input_path)
        except OSError:
            pass
    
        # Set up the GLFW window
        if not glfw.init():
            raise Exception("Could not initialize GLFW")

        window = glfw.create_window(720, 480, "Planar Biped Sim", None, None)
        glfw.make_context_current(window)

        # Create a camera to render the scene
        cam = mujoco.MjvCamera()
        opt = mujoco.MjvOption()

        # set the camera attributes
        cam.distance = self.cam_distance
        cam.elevation = self.cam_elevation
        cam.azimuth = self.cam_azimuth

        # Set up scene and context for rendering
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

        # Initialize a counter for rendering frames
        frame_skip = round(1 / (self.hz_render * self.dt_sim))  # Only render every so number of steps
        step_counter = 0    # Frame counter

        # to keep track of phase
        t_phase_reset = 0.0

        # Main simulation loop
        while (not glfw.window_should_close(window)) and (self.sim_time < self.max_sim_time):

            # update the simulation time
            self.sim_time = self.data.time

            # update phasing variables
            self.update_phase()

            # update the phasing variable i phase completed
            if ((self.sim_time - t_phase_reset) > self.T_SSP) or (self.sim_time == 0.0):

                # update the stance foot
                self.T_phase = 0.0
                t_phase_reset = self.sim_time

                # update the stance foot
                self.update_stance_foot()

            # update the COM state
            self.update_com_state()
            self.update_hlip_state()

            # compute desired outputs
            y_base_des, y_left_des, y_right_des = self.update_output_des()

            # compute the inverse kinematics
            q_left_des, q_right_des = self.compute_inverse_kinematics(y_base_des, y_left_des, y_right_des)

            # compute torques
            q_des = np.array([q_left_des[0], q_left_des[1], q_right_des[0], q_right_des[1]])
            qd_des = np.array([0.0, 0.0, 0.0, 0.0])
            tau = self.compute_torques(q_des, qd_des)
            self.data.ctrl[0] = tau[0][0]
            self.data.ctrl[1] = tau[1][0]
            self.data.ctrl[2] = tau[2][0]
            self.data.ctrl[3] = tau[3][0]

            # compute the forward kinematics
            y_base_act, y_left_act, y_right_act = self.compute_forward_kinematics()

            # update the camera to track the COM
            self.update_camera_to_com(cam)

            # update the visualizations
            self.update_com_visualization()
            self.update_foot_visualization(y_left_des, y_right_des)

            # Log the whole body data
            with open(time_file_path, 'a') as f:
                f.write(f"{self.sim_time}\n")
            with open(pos_file_path, 'a') as f:
                f.write(f"{self.data.qpos[0]},{self.data.qpos[1]},{self.data.qpos[2]},{self.data.qpos[3]},{self.data.qpos[4]},{self.data.qpos[5]},{self.data.qpos[6]}\n")
            with open(vel_file_path, 'a') as f:
                f.write(f"{self.data.qvel[0]},{self.data.qvel[1]},{self.data.qvel[2]},{self.data.qvel[3]},{self.data.qvel[4]},{self.data.qvel[5]},{self.data.qvel[6]}\n")
            with open(tau_file_path, 'a') as f:
                f.write(f"{tau[0][0]},{tau[1][0]},{tau[2][0]},{tau[3][0]}\n")

            # Log the joint data
            with open(q_des_path, 'a') as f:
                f.write(f"{q_des[0][0]},{q_des[1][0]},{q_des[2][0]},{q_des[3][0]}\n")
            with open(q_act_path, 'a') as f:
                f.write(f"{self.data.qpos[3]},{self.data.qpos[4]},{self.data.qpos[5]},{self.data.qpos[6]}\n")

            # Log the desired outputs
            with open(y_des_path, 'a') as f:
                f.write(f"{y_base_des[0][0]},{y_base_des[1][0]},{y_base_des[2][0]},{y_left_des[0][0]},{y_left_des[1][0]},{y_right_des[0][0]},{y_right_des[1][0]}\n")
            with open(y_act_path, 'a') as f:
                f.write(f"{y_base_act[0][0]},{y_base_act[1][0]},{y_base_act[2][0]},{y_left_act[0][0]},{y_left_act[1][0]},{y_right_act[0][0]},{y_right_act[1][0]}\n")

            # Log the phase
            with open(t_phase_path, 'a') as f:
                f.write(f"{self.T_phase}\n")
            with open(stance_path, 'a') as f:
                if self.stance_foot == "L":
                    f.write("1\n")
                elif self.stance_foot == "R":
                    f.write("0\n")

            # Log the ROM state
            with open(rom_state_path, 'a') as f:
                f.write(f"{self.p_rom},{self.v_rom}\n")
            with open(rom_input_path, 'a') as f:
                f.write(f"{self.u}\n")

            # Step the simulation
            mujoco.mj_step(self.model, self.data)

            # Increment the step counter
            step_counter += 1
            if step_counter % frame_skip == 0:
                # Get framebuffer size and create viewport for rendering
                width, height = glfw.get_framebuffer_size(window)
                viewport = mujoco.MjrRect(0, 0, width, height)

                # Update scene for rendering
                mujoco.mjv_updateScene(self.model, self.data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
                mujoco.mjr_render(viewport, self.scene, self.context)

                # Swap buffers and poll events for window
                glfw.swap_buffers(window)

            # Poll for window events like keypress or close
            glfw.poll_events()


####################################################################################
# Main Execution
####################################################################################

if __name__ == "__main__":
    
    # load the config
    with open("../config/biped.yaml", "r") as stream:
        config = yaml.safe_load(stream)

    # create the biped simulation
    biped = BipedSimulation(config)

    # run the simulation
    biped.simulation()
