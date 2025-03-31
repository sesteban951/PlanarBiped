#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "config.h"

#include "mujoco.h"
#include "GLFW/glfw3.h"

#include <Eigen/Dense>

// Mujoco pointers

// Mujoco model pointer
inline mjModel *MJ_MODEL_PTR = NULL;

// Mujoco data pointer
inline mjData *MJ_DATA_PTR = NULL;

// Mujoco contact pointer
inline mjContact *MJ_CONTACT_PTR = NULL;


// Mujoco data structures

// Mujoco camera
inline mjvCamera MJ_CAMERA;

// Mujoco visualization options 
inline mjvOption MJ_OPTIONS;

// Mujoco scene
inline mjvScene MJ_SCENE;

// Mujoco GPU options
inline mjrContext MJ_CONTEXT;

// Mujoco sensor
inline mjfSensor MJ_SENSORS;

// Mujoco visualization options
inline mjvOption vopt;

// Mujoco mouse interactions

// Left mouse click
inline bool BUTTON_LEFT = false;

// Middle mouse click
inline bool BUTTON_MIDDLE = false;

// Right mouse click
inline bool BUTTON_RIGHT =  false;

// X position of last mouse click
inline double BUTTON_LAST_X = 0;

// Y position of last mouse click
inline double BUTTON_LAST_Y = 0;

class Simulator 
{
    public: Simulator();

    public: void Initialize(const char file_name[]);

    public: void UpdateScene();

    public: void SetState(Eigen::Vector<double, N_Q> q_pos);

    public: void SetState(Eigen::Vector<double, N_Q> q_pos, Eigen::Vector<double, N_Q> q_vel);

    public: Eigen::Vector<double, N_Q> GetGeneralizedPosition();

    public: Eigen::Vector<double, N_Q> GetGeneralizedVelocity();

    public: void SetMotorTorques(Eigen::Vector<double, 4> q_tor);

    public: void UpdateStanceFootPosition(Eigen::Vector<double, 3> stf_pos_world_frame);

    public: Eigen::Vector<double, 3> ComputeGlobalCoM(); 

    public: void UpdateGeomPosition(const std::string& geom_name, Eigen::Vector<double, 3> pos);

    public: Eigen::Matrix<double, N_Q, N_Q> GetMassMatrix();

    public: void PropagateDynamics();

    public: void VisualizeSphere(Eigen::Vector<double, 3> position,
                                    double radius,
                                    Eigen::Vector<double, 4> color_rgba);

    public: Eigen::Vector<double, 3> GetTorsoPos();
    public: Eigen::Vector<double, 3> GetTorsoVel();
    public: Eigen::Vector<double, 2> GetStfPos();
    public: Eigen::Vector<double, 2> GetStfVel();
    public: Eigen::Vector<double, 2> GetSwfPos();
    public: Eigen::Vector<double, 2> GetSwfVel();

    public: GLFWwindow* window;

    private: int sensor_torso_pos_idx_;
    private: int sensor_torso_vel_idx_;
    private: int sensor_torso_quat_idx_;
    private: int sensor_torso_ang_vel_idx_;

    private: int sensor_stf_pos_idx_;
    private: int sensor_stf_vel_idx_;

    private: int sensor_swf_pos_idx_;
    private: int sensor_swf_vel_idx_;

    private: Eigen::Vector<double, 3> com_pos_ = Eigen::Vector<double, 3>::Zero();

};

/// \brief The mouse_button function is the callback function for mouse button presses
void MouseButton(GLFWwindow* window, int button, int act, int mods);

/// \brief The mouse_move function is the callback function for mouse movements
void MouseMove(GLFWwindow* window, double xpos, double ypos);

/// \brief The scroll function is the callback function for mouse scrolling presses
void Scroll(GLFWwindow* window, double xoffset, double yoffset);

#endif // SIMULATOR_H
