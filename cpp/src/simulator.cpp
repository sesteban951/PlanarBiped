#include "simulator.h"
#include <iostream>

Simulator::Simulator(){}

void Simulator::Initialize(const char file_name[]) 
{
    char error_msg[1000] = "Failed to load binary model";

    MJ_MODEL_PTR = mj_loadXML(file_name, 0, error_msg, 1000);

    MJ_DATA_PTR = mj_makeData(MJ_MODEL_PTR);

    // Initialize GLFW
    if (!glfwInit())
    {
        mju_error("Could not initialize GLFW");
    }

    // Create a window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Initialize visualization data structures
    mjv_defaultCamera(&MJ_CAMERA);
    mjv_defaultOption(&MJ_OPTIONS);
    mjv_defaultScene(&MJ_SCENE);
    mjr_defaultContext(&MJ_CONTEXT);
    mjv_makeScene(MJ_MODEL_PTR, &MJ_SCENE, 10000);
    mjr_makeContext(MJ_MODEL_PTR, &MJ_CONTEXT, mjFONTSCALE_150);

    // Install GLFW mouse and Keyboard callbacks
    //glfwSetKeyCallback(window, Keyboard);
    //glfwSetKeyCallback(window, KeyVelocityCallback);
    glfwSetCursorPosCallback(window, MouseMove);
    glfwSetMouseButtonCallback(window, MouseButton);
    glfwSetScrollCallback(window, Scroll);

    // Set the rendering options
    mjv_defaultOption(&vopt);

    // Set defualt camera view
    double arr_view[] = {90, -3, 3, 0.012768, -0.000000, 0.54336};
    MJ_CAMERA.azimuth = arr_view[0];
    MJ_CAMERA.elevation = arr_view[1];
    MJ_CAMERA.distance = arr_view[2];
    MJ_CAMERA.lookat[0] = arr_view[3];
    MJ_CAMERA.lookat[1] = arr_view[4];
    MJ_CAMERA.lookat[2] = arr_view[5];

    // Get the sensor indices
    this->sensor_torso_pos_idx_ = MJ_MODEL_PTR->sensor_adr[mj_name2id(MJ_MODEL_PTR, mjOBJ_SENSOR, "torso_pos")];
    this->sensor_torso_vel_idx_ = MJ_MODEL_PTR->sensor_adr[mj_name2id(MJ_MODEL_PTR, mjOBJ_SENSOR, "torso_vel")];
    this->sensor_torso_quat_idx_ = MJ_MODEL_PTR->sensor_adr[mj_name2id(MJ_MODEL_PTR, mjOBJ_SENSOR, "torso_quat")];
    this->sensor_torso_ang_vel_idx_ = MJ_MODEL_PTR->sensor_adr[mj_name2id(MJ_MODEL_PTR, mjOBJ_SENSOR, "torso_ang_vel")];

    this->sensor_stf_pos_idx_ = MJ_MODEL_PTR->sensor_adr[mj_name2id(MJ_MODEL_PTR, mjOBJ_SENSOR, "stf_pos")];
    this->sensor_stf_vel_idx_ = MJ_MODEL_PTR->sensor_adr[mj_name2id(MJ_MODEL_PTR, mjOBJ_SENSOR, "stf_vel")];

    this->sensor_swf_pos_idx_ = MJ_MODEL_PTR->sensor_adr[mj_name2id(MJ_MODEL_PTR, mjOBJ_SENSOR, "swf_pos")];
    this->sensor_swf_vel_idx_ = MJ_MODEL_PTR->sensor_adr[mj_name2id(MJ_MODEL_PTR, mjOBJ_SENSOR, "swf_vel")];
}

Eigen::Vector<double, 3> Simulator::GetTorsoPos()
{
    Eigen::Vector<double, 3> torso_pos = Eigen::Vector<double, 3>::Zero();
    torso_pos(0) = MJ_DATA_PTR->sensordata[this->sensor_torso_pos_idx_ + 0];
    //torso_pos(1) = MJ_DATA_PTR->sensordata[this->sensor_torso_pos_idx_ * 3 + 1];
    torso_pos(1) = MJ_DATA_PTR->sensordata[this->sensor_torso_pos_idx_ + 2];

    return torso_pos;
}

Eigen::Vector<double, 3> Simulator::GetTorsoVel()
{
    Eigen::Vector<double, 3> torso_vel = Eigen::Vector<double, 3>::Zero();
    torso_vel(0) = MJ_DATA_PTR->sensordata[this->sensor_torso_vel_idx_ + 0];
    //torso_vel(1) = MJ_DATA_PTR->sensordata[this->sensor_torso_vel_idx_ * 3 + 1];
    torso_vel(1) = MJ_DATA_PTR->sensordata[this->sensor_torso_vel_idx_ + 2];

    torso_vel(2) = MJ_DATA_PTR->sensordata[this->sensor_torso_ang_vel_idx_ + 1];

    return torso_vel;
}

Eigen::Vector<double, 2> Simulator::GetStfPos()
{
    Eigen::Vector<double, 2> stf_pos = Eigen::Vector<double, 2>::Zero();
    stf_pos(0) = MJ_DATA_PTR->sensordata[this->sensor_stf_pos_idx_ + 0];
    stf_pos(1) = MJ_DATA_PTR->sensordata[this->sensor_stf_pos_idx_ + 2];

    return stf_pos;
}

Eigen::Vector<double, 2> Simulator::GetStfVel()
{
    Eigen::Vector<double, 2> stf_vel = Eigen::Vector<double, 2>::Zero();
    stf_vel(0) = MJ_DATA_PTR->sensordata[this->sensor_stf_vel_idx_ + 0];
    stf_vel(1) = MJ_DATA_PTR->sensordata[this->sensor_stf_vel_idx_ + 2];

    return stf_vel;
}

Eigen::Vector<double, 2> Simulator::GetSwfPos()
{
    Eigen::Vector<double, 2> swf_pos = Eigen::Vector<double, 2>::Zero();
    swf_pos(0) = MJ_DATA_PTR->sensordata[this->sensor_swf_pos_idx_ + 0];
    swf_pos(1) = MJ_DATA_PTR->sensordata[this->sensor_swf_pos_idx_ + 2];

    return swf_pos;
}

Eigen::Vector<double, 2> Simulator::GetSwfVel()
{
    Eigen::Vector<double, 2> swf_vel = Eigen::Vector<double, 2>::Zero();
    swf_vel(0) = MJ_DATA_PTR->sensordata[this->sensor_swf_vel_idx_ + 0];
    swf_vel(1) = MJ_DATA_PTR->sensordata[this->sensor_swf_vel_idx_ + 2];

    return swf_vel;
}

void Simulator::UpdateScene()
{
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(MJ_MODEL_PTR, MJ_DATA_PTR, &(MJ_OPTIONS), NULL, &MJ_CAMERA, mjCAT_ALL, &MJ_SCENE);

    // Update the visualization
    mjr_render(viewport, &MJ_SCENE, &MJ_CONTEXT);

    // // // Make the camera follow the robot
    // MJ_CAMERA.lookat[0] = MJ_DATA_PTR->qpos[0];
    // MJ_CAMERA.lookat[1] = MJ_DATA_PTR->qpos[1];
    // MJ_CAMERA.lookat[2] = MJ_DATA_PTR->qpos[2];

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void Simulator::SetState(Eigen::Vector<double, N_Q> q_pos)
{
    // Set the joint positions in the Mujoco data structure
    for(int i = 0; i < N_Q; i++)
    {
        if (i < MJ_MODEL_PTR->nq) // Ensure we don't go out of bounds
        {
            MJ_DATA_PTR->qpos[i] = q_pos(i);
            MJ_DATA_PTR->qvel[i] = 0;
        }
    }

    // Update the data structure to reflect the new state
    mj_forward(MJ_MODEL_PTR, MJ_DATA_PTR);
}

void Simulator::SetState(Eigen::Vector<double, N_Q> q_pos, Eigen::Vector<double, N_Q> q_vel)
{
    // Set the joint positions in the Mujoco data structure
    for(int i = 0; i < N_Q; i++)
    {
        if (i < MJ_MODEL_PTR->nq) // Ensure we don't go out of bounds
        {
            MJ_DATA_PTR->qpos[i] = q_pos(i);
            MJ_DATA_PTR->qvel[i] = q_vel(i);
        }
    }

    // Update the data structure to reflect the new state
    mj_forward(MJ_MODEL_PTR, MJ_DATA_PTR);
}

void Simulator::SetMotorTorques(Eigen::Vector<double, 4> q_tor)
{
    // Set the motor torques in the Mujoco data structure
    for(int i = 0; i < 4; i++)
    {
        if (i < MJ_MODEL_PTR->nu) // Ensure we don't go out of bounds
        {
            MJ_DATA_PTR->ctrl[i] = q_tor(i);
        }
        else
        {
            std::cerr << "Error: Invalid motor index" << std::endl;
        }
    }
}

Eigen::Vector<double, N_Q> Simulator::GetGeneralizedPosition()
{
    Eigen::Vector<double, N_Q> q_pos;

    for(int i = 0; i < N_Q; i++)
    {
        if (i < MJ_MODEL_PTR->nq) // Ensure we don't go out of bounds
        {
            q_pos(i) = MJ_DATA_PTR->qpos[i];
        }
    }

    return q_pos;
}

Eigen::Vector<double, N_Q> Simulator::GetGeneralizedVelocity()
{
    Eigen::Vector<double, N_Q> q_vel;

    for(int i = 0; i < N_Q; i++)
    {
        if (i < MJ_MODEL_PTR->nq) // Ensure we don't go out of bounds
        {
            q_vel(i) = MJ_DATA_PTR->qvel[i];
        }
    }

    return q_vel;
}

void Simulator::UpdateStanceFootPosition(Eigen::Vector<double, 3> stf_pos_world_frame) 
{
    int body_id = mj_name2id(MJ_MODEL_PTR, mjOBJ_BODY, "stance_foot");
    if (body_id == -1) {
        std::cerr << "Error: stance_foot body not found!" << std::endl;
        return;
    }

    int mocap_id = MJ_MODEL_PTR->body_mocapid[body_id];
    if (mocap_id == -1) {
        std::cerr << "Error: stance_foot is not a mocap body!" << std::endl;
        return;
    }

    // Update mocap position
    MJ_DATA_PTR->mocap_pos[mocap_id * 3] = stf_pos_world_frame[0];
    MJ_DATA_PTR->mocap_pos[mocap_id * 3 + 1] = stf_pos_world_frame[1];
    MJ_DATA_PTR->mocap_pos[mocap_id * 3 + 2] = stf_pos_world_frame[2];

    // Apply update
    mj_forward(MJ_MODEL_PTR, MJ_DATA_PTR);
}

void MouseButton(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    BUTTON_LEFT =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    BUTTON_MIDDLE = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    BUTTON_RIGHT =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &BUTTON_LAST_X, &BUTTON_LAST_Y);
}

void MouseMove(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !BUTTON_LEFT && !BUTTON_MIDDLE && !BUTTON_RIGHT )
        return;

    // compute mouse displacement, save
    double dx = xpos - BUTTON_LAST_X;
    double dy = ypos - BUTTON_LAST_Y;
    BUTTON_LAST_X = xpos;
    BUTTON_LAST_Y = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( BUTTON_RIGHT )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( BUTTON_LEFT )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(MJ_MODEL_PTR, action, dx/height, dy/height, &MJ_SCENE, &MJ_CAMERA);
}

void Scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(MJ_MODEL_PTR, mjMOUSE_ZOOM, 0, -0.05*yoffset, &MJ_SCENE, &MJ_CAMERA);
}