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

    public: Eigen::Vector<double, N_Q> GetGeneralizedPosition();

    public: Eigen::Vector<double, N_Q> GetGeneralizedVelocity();

    public: void SetMotorTorques(Eigen::Vector<double, 4> q_tor);

    public: void UpdateStanceFootPosition(Eigen::Vector<double, 3> stf_pos_world_frame);

    public: GLFWwindow* window;

};

/// \brief The mouse_button function is the callback function for mouse button presses
void MouseButton(GLFWwindow* window, int button, int act, int mods);

/// \brief The mouse_move function is the callback function for mouse movements
void MouseMove(GLFWwindow* window, double xpos, double ypos);

/// \brief The scroll function is the callback function for mouse scrolling presses
void Scroll(GLFWwindow* window, double xoffset, double yoffset);

#endif // SIMULATOR_H
