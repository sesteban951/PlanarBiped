#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "config.h"

#include <cmath> 

// Include eigen
#include <Eigen/Dense>

class Controller 
{
    public: Controller();

    Eigen::Vector<double, N_Q> SolveIK(Eigen::Vector<double, N_OUTPUTS> y_ref);

    Eigen::Vector<double, N_OUTPUTS> CalculateOutputsInStanceFootFrame(Eigen::Vector<double, N_Q> q_pos);

    Eigen::Vector<double, N_OUTPUTS> CalculateOutputsInWorldFrame(Eigen::Vector<double, N_Q> q_pos);

    void UpdateController(Eigen::Vector<double, N_Q> q_pos, 
                            Eigen::Vector<double, N_Q> q_vel,
                            double t_step);

    // Model parameters
    private: double l_thigh_ = 0.5;
    private: double l_shin_ = 0.5;

    // Control parameters
    private: double com_pos_z_ref_ = 0.7;

    private: double T_SSP_ = 0.5;
    private: double T_DSP_ = 0.2;

    private: double p_x_stf_world_frame_ = 0.0;

};

#endif // CONTROLLER_H