#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "config.h"

#include <cmath> 

// Include eigen
#include <Eigen/Dense>

class Controller 
{
    public: Controller();
    
    void computeControl();  // Example function

    Eigen::Vector<double, N_Q> SolveIK(Eigen::Vector<double, N_OUTPUTS> y_ref);

    // Model parameters
    private: double l_thigh_ = 0.5;
    private: double l_shin_ = 0.5;

    // Control parameters
    private: double com_pos_z_ref_ = 0.7;

    private: double T_SSP_ = 0.5;
    private: double T_DSP_ = 0.2;

};

#endif // CONTROLLER_H