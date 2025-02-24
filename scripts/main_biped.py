
# standard library
import numpy as np
import os

# mujoco stuff
import mujoco
import glfw

# bezier stuff
import bezier

####################################################################################
# Biped Simulation
####################################################################################

class BipedSimulation:

    def __init__(self):
        
        # load the model
        xml_path = "../models/biped/biped.xml"
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # initial state
        qpos = np.array([0.0, 1.1,                # position x and z 
                         0.5,                     # theta body
                         0.0, 0.0, 0.0, 0.0])  # left thigh, left knee, right thigh, right knee
        qvel = np.array([0.0, 0.0, 
                         0.0, 
                         0.0, 0.0,  0.0, 0.0])  
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel

        # geomtry id
        self.left_foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "foot_left")
        self.right_foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "foot_right")
        
        # sim parameters
        self.sim_time = 0.0
        self.max_sim_time = 10.0
        self.dt_sim = self.model.opt.timestep
        self.hz_render = 1000

        # model parameters
        self.z_foot_offset = 0.075 # (foot is round)
        self.l1 = 0.5 # length of the thigh
        self.l2 = 0.5 # length of the shank

        # center of mass state
        self.p_com = None
        self.v_com = None

        # ROM state
        self.p_rom = None
        self.v_rom = None

        # desired COM state
        self.p_rom_des = 0
        self.v_rom_des = 0

        # phasing variables
        self.T_SSP = 0.28
        self.T_phase = 0.0
        self.num_steps = 0
        self.stance_foot = None

        # desired height
        self.theta_des = -0.2  # desired torso angle
        self.z_0 = 0.9        # LIP constant height
        self.z_apex = 0.15     # foot apex height

        # foot position variables
        self.p_stance_prev = None # previous stance foot position
        self.p_stance = None      # current stance foot position
        self.p_swing = None       # current swing foot position
        self.u = None             # foot placement w.r.t to stance foot

        # current joint state solution
        self.q_ik = np.zeros((4, 1))

        # Raibert gains
        self.kp_raibert = -1.0
        self.kd_raibert = -0.

        # low level joint gains
        self.kp_H = 250
        self.kd_H = 10
        self.kp_K = 250
        self.kd_K = 10

    ############################################### ROM ######################################

    # update the phasing variable
    def update_phase(self):

        # compute the number of steps taken so far
        self.num_steps = int(self.sim_time / self.T_SSP)

        # update the phase variable
        self.T_phase = self.sim_time - self.num_steps * self.T_SSP

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
            self.p_stance_prev = np.array([y_right_W[0], [self.z_foot_offset]]).reshape(2, 1)
            self.p_stance = np.array([y_left_W[0], [self.z_foot_offset]]).reshape(2, 1)

        elif self.stance_foot == "R":
            self.p_stance_prev = np.array([y_left_W[0], [self.z_foot_offset]]).reshape(2, 1)
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

        # TODO: computing this the wrong way I think 
        u_pos = self.kp_raibert * (self.p_rom_des - self.p_rom)
        u_vel = self.kd_raibert * (self.v_rom_des - self.v_rom)

        self.u = u_pos + u_vel
        

    ############################################### KINEMATICS ######################################

    # compute swing foot based on foot placement and bezier curve
    def compute_swing_foot_pos(self):

        # compute the foot placement
        self.compute_foot_placement()

        # initial and end points
        x0 = self.p_stance_prev[0][0]
        xm = (self.p_stance[0][0] + self.u) /2
        xf = self.p_stance[0][0] + self.u
        
        z0 = self.z_foot_offset
        zm = self.z_apex * (16/5) + self.z_foot_offset
        zf = self.z_foot_offset

        # bezier curve points
        x_pts = np.array([x0, x0, x0, xm, xf, xf, xf]) # x Bezier points
        z_pts = np.array([z0, z0, z0, zm, zf, zf, zf]) # z Bezier points
        P = np.array([x_pts, z_pts])

        # want to compute this at normalized time
        t = self.T_phase / self.T_SSP

        # build the bezier curve
        nodes = np.asfortranarray(P)
        curve = bezier.Curve(nodes, degree=(P.shape[1]-1))
        
        # evaulate the bezier curve at time t
        p_swing = curve.evaluate(t)
        
        return p_swing

    # update the desired outputs
    def update_output_des(self):
        
        # compute the desired base outputs
        y_base_W, _, _ = self.compute_forward_kinematics()
        y_base_W[1] = self.z_0 + self.z_foot_offset
        y_base_W[2] = self.theta_des
        y_base_des_W = y_base_W
        
        # TODO: probably dont use the base directly

        # define the desired foot outputs
        p_swing_W = self.compute_swing_foot_pos()
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

        # query the location of the feet
        # left_pos = self.data.geom_xpos[self.left_foot_id]
        # right_pos = self.data.geom_xpos[self.right_foot_id]
        # y_left_W = np.array([left_pos[0], left_pos[2]]).reshape(2, 1)
        # y_right_W = np.array([right_pos[0], right_pos[2]]).reshape(2, 1)

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

        # unpack the desired position
        x_L = p_left_B[0][0]
        z_L = p_left_B[1][0]
        x_R = p_right_B[0][0]
        z_R = p_right_B[1][0]

        # compute the angles
        # c_L = (x_L**2 + z_L**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        # s_L = -np.sqrt(1 - c_L**2)
        # q_KL = np.arctan2(s_L, c_L)

        # c_R = (x_R**2 + z_R**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        # s_R = -np.sqrt(1 - c_R**2)
        # q_KR = np.arctan2(s_R, c_R)

        # q_HL = np.arctan2(z_L, x_L) - np.arctan2(self.l2 * np.sin(q_KL), self.l1 + self.l2 * np.cos(q_KL)) + np.pi/2 
        # q_HR = np.arctan2(z_R, x_R) - np.arctan2(self.l2 * np.sin(q_KR), self.l1 + self.l2 * np.cos(q_KR)) + np.pi/2

        # knee angle (Left)
        L = np.sqrt(x_L**2 + z_L**2)
        gamma = np.arccos((L**2 - self.l1**2 - self.l2**2) / (-2 * self.l1 * self.l2))
        q_KL = gamma - np.pi

        # hip angle (Left)
        beta = np.arctan2(x_L, -z_L)
        alpha = np.arccos((self.l2**2 - self.l1**2 - L**2) / (-2 * self.l1 * L))
        q_HL = beta + alpha

        # knee angle (Right)
        L = np.sqrt(x_R**2 + z_R**2)
        gamma = np.arccos((L**2 - self.l1**2 - self.l2**2) / (-2 * self.l1 * self.l2))
        q_KR = gamma - np.pi
        
        # hip angle (Right)
        beta = np.arctan2(x_R, -z_R)
        alpha = np.arccos((self.l2**2 - self.l1**2 - L**2) / (-2 * self.l1 * L))
        q_HR = beta + alpha

        # pack the positions
        q_base = np.array([p_base_W[0], p_base_W[1], theta_des]).reshape(3, 1)
        q_left = np.array([q_HL, q_KL]).reshape(2, 1)
        q_right = np.array([q_HR, q_KR]).reshape(2, 1)

        return q_base, q_left, q_right
    
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

    def update_com_visualization(self):
        p_com_flat = np.ravel(self.p_com)
        com_pos = np.array([p_com_flat[0], 0.0, p_com_flat[1]])
        com_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "com_vis")
        if com_geom_id != -1:
            self.model.geom_pos[com_geom_id] = com_pos  # Update position

    # Function to update the camera position to track the center of mass (CoM)
    def update_camera_to_com(self, cam):

        # Set camera parameters to track the CoM
        cam.lookat[:] = [self.p_com[0][0], 0, self.p_com[1][0]]  # Make the camera look at the CoM
        cam.distance = 3.0  # Distance from the CoM (adjust as needed)
        cam.elevation = -10  # Camera elevation angle (adjust as needed)
        cam.azimuth = 90  # Camera azimuth angle (adjust as needed)

    ############################################### SIMULATION ######################################

    # run the simulation
    def simulation(self):
        
        # csv data path
        time_file_path = "../data/time.csv"
        pos_file_path = "../data/pos.csv"
        vel_file_path = "../data/vel.csv"
        tau_file_path = "../data/tau.csv"

        q_des_path = "../data/q_des.csv"
        q_act_path = "../data/q_act.csv"

        # remove the data files if they exist
        try:
            os.remove(time_file_path)
            os.remove(pos_file_path)
            os.remove(vel_file_path)
            os.remove(tau_file_path)
            os.remove(q_des_path)
            os.remove(q_act_path)
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
            if (self.sim_time - t_phase_reset >= self.T_SSP) or (self.sim_time == 0.0):
                
                # update the stance foot
                self.update_stance_foot()

                # update the stance foot
                t_phase_reset = self.sim_time

            # update the COM state
            self.update_com_state()
            self.update_hlip_state()

            # compute desired outputs
            # y_base_des, y_left_des, y_right_des = self.update_output_des()

            # compute the inverse kinematics
            y_base_des, y_left_des, y_right_des = self.compute_forward_kinematics()
            
            left_foot_pos = self.data.geom_xpos[self.left_foot_id]
            right_foot_pos = self.data.geom_xpos[self.right_foot_id]

            y_left_des[0] = left_foot_pos[0]
            y_left_des[1] = left_foot_pos[2]
            y_right_des[0] = right_foot_pos[0]
            y_right_des[1] = right_foot_pos[2]

            _, q_left_des, q_right_des = self.compute_inverse_kinematics(y_base_des, y_left_des, y_right_des)

            q_left_act = np.array([self.data.qpos[3], self.data.qpos[4]]).reshape(2, 1)
            q_right_act = np.array([self.data.qpos[5], self.data.qpos[6]]).reshape(2, 1)

            print(f"{np.linalg.norm(q_left_des - q_left_act):.4f}")
            print(f"{np.linalg.norm(q_right_des - q_right_act):.4f}")

            # # compute torques
            # q_des = np.array([q_left_des[0], q_left_des[1], q_right_des[0], q_right_des[1]])
            # qd_des = np.array([0.0, 0.0, 0.0, 0.0])

            # # apply the torques
            # tau = self.compute_torques(q_des, qd_des)
            # self.data.ctrl[0] = tau[0][0]
            # self.data.ctrl[1] = tau[1][0]
            # self.data.ctrl[2] = tau[2][0]
            # self.data.ctrl[3] = tau[3][0]

            # fixed joint state
            q_des  = np.array([0.5, -0.2, -0.3, -0.2])  # Initial joint positions
            qd_des = np.array([0.0, 0.0,  0.0, 0.0])   # Initial joint positions
            tau = self.compute_torques(q_des, qd_des)
            self.data.ctrl[0] = tau[0]
            self.data.ctrl[1] = tau[1]
            self.data.ctrl[2] = tau[2]
            self.data.ctrl[3] = tau[3]

            # update the camera to track the COM
            self.update_camera_to_com(cam)

            # update the COM visualization
            self.update_com_visualization()

            # Log the sim data
            # with open(time_file_path, 'a') as f:
            #     f.write(f"{self.sim_time}\n")
            # with open(pos_file_path, 'a') as f:
            #     f.write(f"{self.data.qpos[0]},{self.data.qpos[1]},{self.data.qpos[2]},{self.data.qpos[3]},{self.data.qpos[4]},{self.data.qpos[5]},{self.data.qpos[6]}\n")
            # with open(vel_file_path, 'a') as f:
            #     f.write(f"{self.data.qvel[0]},{self.data.qvel[1]},{self.data.qvel[2]},{self.data.qvel[3]},{self.data.qvel[4]},{self.data.qvel[5]},{self.data.qvel[6]}\n")
            # with open(tau_file_path, 'a') as f:
            #     f.write(f"{tau[0]},{tau[1]},{tau[2]},{tau[3]}\n")

            # with open(q_des_path, 'a') as f:
            #     f.write(f"{q_des[0][0]},{q_des[1][0]},{q_des[2][0]},{q_des[3][0]}\n")
            # with open(q_act_path, 'a') as f:
            #     f.write(f"{self.data.qpos[3]},{self.data.qpos[4]},{self.data.qpos[5]},{self.data.qpos[6]}\n")

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
    
    biped = BipedSimulation()

    biped.simulation()


