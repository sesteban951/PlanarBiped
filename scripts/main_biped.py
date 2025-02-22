
# standard library
import numpy as np
import time
import os

# mujoco stuff
import mujoco
import glfw

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
                         0.0,                     # theta body
                         0.5, -0.2, -0.3, -0.2])  # left thigh, left knee, right thigh, right knee
        qvel = np.array([0.0, 0.0, 
                         0.0, 
                         0.0, 0.0,  0.0, 0.0])  

        # set the initial state
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel

        # sim parameters
        self.sim_time = 0.0
        self.max_sim_time = 15.0
        self.dt_sim = self.model.opt.timestep
        self.hz_render = 40

        # center of mass state
        self.p_com = None
        self.v_com = None

        # ROM state
        self.p_rom = None
        self.v_rom = None

        # phasing variables
        self.T_SSP = 0.35
        self.T_phase = 0.0
        self.num_steps = 0
        self.stance_foot = None

        # desired height
        self.theta_des = 0.0  # desired torso angle
        self.z_0 = 1.0        # LIP constant height
        self.z_apex = 0.2     # foot apex height

        # output variables
        self.p_stance = None
        self.p_swing = None

        # low level joint gains
        self.kp_H = 100
        self.kd_H = 10
        self.kp_K = 150
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
    def update_foot_role(self):

        # TODO: based on the phase variable, decide which foot is swing and stance

        pass

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

        # update teh COM state
        self.p_com = np.array([com[0], com[2]]).reshape(2, 1)
        self.v_com = np.array([self.data.qvel[0], self.data.qvel[2]]).reshape(2, 1)

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

    # compute the foot placement
    def compute_foot_placement(self):

        # Raibert gains
        kp = 1.0
        kd = 0.1

        # compute the foot placement
        p_rom_des = 0
        v_rom_des = 0
        u = kp * (p_rom_des - self.p_rom) + kd * (v_rom_des - self.v_rom)

        return u

    ############################################### KINEMATICS ######################################

    # compute bezier curve value
    def compute_bezier(self, u):

        # TODO: compute the bezier curve value based on stance and required foot step

        pass

    # update the desired outputs
    def update_output_des(self):

        # update the desired outputs
        
        # compute the desired foot placement
        u = self.compute_foot_placement()


        # TODO: defined the desired outputs

        pass

    # compute the forward kinematics in WORLD Frame
    def compute_forward_kinematics(self):

        # lengths of the legs
        l1 = 0.5 # length of the thigh
        l2 = 0.5 # length of the shank

        # unpack the base position
        p_base_W = np.array([self.data.qpos[0], self.data.qpos[1]]).reshape(2, 1)
        theta_W = self.data.qpos[2]
        R = np.array([[np.cos(theta_W), -np.sin(theta_W)],
                    [np.sin(theta_W),  np.cos(theta_W)]])

        # unpack the joint angles
        q_HL = self.data.qpos[3]
        q_KL = self.data.qpos[4]
        q_HR = self.data.qpos[5]
        q_KR = self.data.qpos[6]

        # compute the positions of the feet relative to the base frame
        p_left_B = np.array([l1 * np.sin(q_HL) + l2 * np.sin(q_HL + q_KL),
                            -l1 * np.cos(q_HL) - l2 * np.cos(q_HL + q_KL)]).reshape(2, 1)
        p_right_B = np.array([l1 * np.sin(q_HR) + l2 * np.sin(q_HR + q_KR),
                            -l1 * np.cos(q_HR) - l2 * np.cos(q_HR + q_KR)]).reshape(2, 1)
        
        # compute the outputs
        y_base_W = np.array([p_base_W[0], p_base_W[1], [theta_W]]).reshape(3, 1)
        y_left_W = p_base_W + R @ p_left_B
        y_right_W = p_base_W + R @ p_right_B
        
        return y_base_W, y_left_W, y_right_W

    # compute inverse kineamtics given feet position in world frame
    def compute_inverse_kinematics(self, y_base_W, y_left_W, y_right_W):

        # lengths of the legs
        l1 = 0.5 # length of the thigh
        l2 = 0.5 # length of the shank

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
        c_L = (x_L**2 + z_L**2 - l1**2 - l2**2) / (2 * l1 * l2)
        s_L = -np.sqrt(1 - c_L**2)
        q_KL = np.arctan2(s_L, c_L)

        c_R = (x_R**2 + z_R**2 - l1**2 - l2**2) / (2 * l1 * l2)
        s_R = -np.sqrt(1 - c_R**2)
        q_KR = np.arctan2(s_R, c_R)

        q_HL = np.arctan2(z_L, x_L) - np.arctan2(l2 * np.sin(q_KL), l1 + l2 * np.cos(q_KL)) + np.pi/2 
        q_HR = np.arctan2(z_R, x_R) - np.arctan2(l2 * np.sin(q_KR), l1 + l2 * np.cos(q_KR)) + np.pi/2

        # pack the positions
        q_base = np.array([p_base_W[0], p_base_W[1], theta_des]).reshape(3, 1)
        q_left = np.array([q_HL, q_KL]).reshape(2, 1)
        q_right = np.array([q_HR, q_KR]).reshape(2, 1)

        return q_base, q_left, q_right
    
    ############################################### LOW LEVEL ######################################

    # Function to compute torques based on desired position and velocity
    def compute_torques(self,q_des, qd_des):

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

        # remove the data files if they exist
        try:
            os.remove(time_file_path)
            os.remove(pos_file_path)
            os.remove(vel_file_path)
            os.remove(tau_file_path)
        except OSError:
            pass
    
        # Set up the GLFW window
        if not glfw.init():
            raise Exception("Could not initialize GLFW")

        window = glfw.create_window(1920, 1080, "Planar Biped Sim", None, None)
        glfw.make_context_current(window)

        # Create a camera to render the scene
        cam = mujoco.MjvCamera()
        opt = mujoco.MjvOption()

        # Set up scene and context for rendering
        scene = mujoco.MjvScene(self.model, maxgeom=10000)
        context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

        # Initialize a counter for rendering frames
        frame_skip = round(1 / (self.hz_render * self.dt_sim))  # Only render every so number of steps
        step_counter = 0    # Frame counter
        print("Frame skip: ", frame_skip)

        # Main simulation loop
        while (not glfw.window_should_close(window)) and (self.sim_time < self.max_sim_time):

            # update the simulation time
            self.sim_time = self.data.time

            # update the phasing variable
            self.update_phase()

            # update the COM state
            self.update_com_state()
            self.update_hlip_state()

            # compute desired outputs
            self.update_output_des()

            # desired joint state TODO: will replace this with outputs
            q_des  = np.array([0.5, -0.2, -0.3, -0.2])  # Initial joint positions
            qd_des = np.array([0.0, 0.0,  0.0, 0.0])   # Initial joint positions

            # compute torques
            tau = self.compute_torques(q_des, qd_des)

            # apply the torques
            self.data.ctrl[0] = tau[0]
            self.data.ctrl[1] = tau[1]
            self.data.ctrl[2] = tau[2]
            self.data.ctrl[3] = tau[3]

            # update the camera to track the COM
            self.update_camera_to_com(cam)

             # Log thesim data
            with open(time_file_path, 'a') as f:
                f.write(f"{self.sim_time}\n")
            with open(pos_file_path, 'a') as f:
                f.write(f"{self.data.qpos[0]},{self.data.qpos[1]},{self.data.qpos[2]},{self.data.qpos[3]},{self.data.qpos[4]},{self.data.qpos[5]},{self.data.qpos[6]}\n")
            with open(vel_file_path, 'a') as f:
                f.write(f"{self.data.qvel[0]},{self.data.qvel[1]},{self.data.qvel[2]},{self.data.qvel[3]},{self.data.qvel[4]},{self.data.qvel[5]},{self.data.qvel[6]}\n")
            with open(tau_file_path, 'a') as f:
                f.write(f"{tau[0]},{tau[1]},{tau[2]},{tau[3]}\n")

            # Step the simulation
            mujoco.mj_step(self.model, self.data)

            # Increment the step counter
            step_counter += 1
            if step_counter % frame_skip == 0:
                # Get framebuffer size and create viewport for rendering
                width, height = glfw.get_framebuffer_size(window)
                viewport = mujoco.MjrRect(0, 0, width, height)

                # Update scene for rendering
                mujoco.mjv_updateScene(self.model, self.data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
                mujoco.mjr_render(viewport, scene, context)

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


