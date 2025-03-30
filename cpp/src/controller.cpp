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

Eigen::Vector<double, N_OUTPUTS> Controller::CalculateOutputsInStanceFootFrame(Eigen::Vector<double, N_Q> q_pos)
{
    // Get the joint position values
    double q_stf_ankle = (0);
    double q_stf_knee = (1);
    double q_stf_hip = (2);
    double q_swf_hip = (3);
    double q_swf_knee = (4); 

    // Compute the outputs

    // Compute the base position in the world frame
    double p_x_com_stf_frame = this->l_shin_ * sin(q_stf_ankle) + this->l_thigh_ * sin(q_stf_ankle + q_stf_knee);
    double p_z_com_stf_frame = this->l_shin_ * cos(q_stf_ankle) + this->l_thigh_ * cos(q_stf_ankle + q_stf_knee);

    // Compute the swing foot position in the world frame
    double p_x_swf_stf_frame = p_x_com_stf_frame + this->l_thigh_ * sin(q_stf_ankle + q_stf_knee - q_stf_hip) + this->l_shin_ * sin(q_stf_ankle + q_stf_knee - q_stf_hip - q_swf_hip - q_swf_knee);
    double p_z_swf_stf_frame = p_z_com_stf_frame + this->l_thigh_ * cos(q_stf_ankle + q_stf_knee - q_stf_hip) + this->l_shin_ * cos(q_stf_ankle + q_stf_knee - q_stf_hip - q_swf_hip - q_swf_knee);

    // Compute the base pitch in the world frame
    double pitch_com_stf_frame = q_stf_ankle + q_stf_knee - q_stf_hip;

    // Store the the outputs in a vector
    Eigen::Vector<double, N_OUTPUTS> y_stf_frame;
    y_stf_frame(0) = p_x_com_stf_frame;
    y_stf_frame(1) = p_z_com_stf_frame;
    y_stf_frame(2) = pitch_com_stf_frame;
    y_stf_frame(3) = p_x_swf_stf_frame;
    y_stf_frame(4) = p_z_swf_stf_frame;

    // Return the outputs
    return y_stf_frame;
}

Eigen::Vector<double, N_OUTPUTS> Controller::CalculateOutputsInWorldFrame(Eigen::Vector<double, N_Q> q_pos)
{
    Eigen::Vector<double, N_OUTPUTS> y_stf_frame = CalculateOutputsInStanceFootFrame(q_pos);

    // Calculate the outputs in the world frame
    Eigen::Vector<double, N_OUTPUTS> y_world_frame;

    // Add the stance foot offset to the outputs
    y_world_frame(OutputIDX::COM_POS_X) = y_stf_frame(0) + this->p_x_stf_world_frame_;
    y_world_frame(OutputIDX::SWF_POS_X) = y_stf_frame(3) + this->p_x_stf_world_frame_;

    // Return the outputs in the world frame
    return y_world_frame;
}

void Controller::UpdateController(Eigen::Vector<double, N_Q> q_pos, 
                                  Eigen::Vector<double, N_Q> q_vel,
                                  double t_step)
{
    // This function can be used to update the controller based on the current state
    // For now, it does nothing but can be implemented for advanced control strategies
    // Example: PID control, LQR, etc.

    // Note: This is a placeholder for future control logic
    // You can implement your control strategy here based on q_pos and q_vel
}