#include "controller.h"
#include <iostream>

Controller::Controller(){}

Eigen::Vector<double, N_Q> Controller::SolveIK(Eigen::Vector<double, N_OUTPUTS> y_ref) 
{
    double com_pos_x = y_ref(0);
    double com_pos_z = y_ref(1);
    double com_theta = y_ref(2);
    double swf_pos_x = y_ref(3);
    double swf_pos_z = y_ref(4);

    double L_stf = sqrt(com_pos_x * com_pos_x + com_pos_z * com_pos_z);

    double beta_stf = acos((this->l_thigh_ * this->l_thigh_ + this->l_shin_ * this->l_shin_ - L_stf * L_stf) / (2.0 * this->l_thigh_ * this->l_shin_));

    double q_stf_knee = beta_stf - M_PI;

    double mu_stf = atan2(com_pos_x, com_pos_z);

    double gamma_stf = asin((this->l_shin_ / L_stf) * sin(beta_stf)); 

    double q_stf_ankle = mu_stf + gamma_stf;

    double q_stf_hip = -(q_stf_ankle + q_stf_knee) + com_theta;

    // Swing leg
    double L_swf = sqrt((swf_pos_x - com_pos_x) * (swf_pos_x - com_pos_x) + (swf_pos_z - com_pos_z) * (swf_pos_z - com_pos_z));

    double beta_swf = acos((this->l_shin_ * this->l_shin_ + this->l_thigh_ * this->l_thigh_ - L_swf * L_swf) / (2.0 * this->l_shin_ * this->l_thigh_));

    double q_swf_knee = M_PI - beta_swf;

    double gamma_swf = asin((this->l_thigh_ / L_swf) * sin(beta_swf));

    double alpha_swf = atan2(-(swf_pos_x - com_pos_x), (com_pos_z - swf_pos_z));
    
    double q_swf_hip = alpha_swf - com_theta - gamma_swf;

    
    // Store the joint angles
    Eigen::Vector<double, N_Q> q_pos;
    q_pos(0) = q_stf_ankle; // Ankle angle for the stance leg
    q_pos(1) = q_stf_knee;  // Knee angle for the stance leg
    q_pos(2) = q_stf_hip;   // Hip angle for the stance leg
    q_pos(3) = q_swf_hip;   // Hip angle for the swing leg
    q_pos(4) = q_swf_knee;  // Knee angle for the swing leg

    return q_pos;
}