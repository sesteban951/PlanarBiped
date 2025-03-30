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

Eigen::Vector<double, N_Q> Controller::SolveIKDerivative(Eigen::Vector<double, N_OUTPUTS> y_dot, Eigen::Vector<double, N_Q> q_pos)
{
    Eigen::Matrix<double, N_Q, N_OUTPUTS> J = this->ComputeOutputJacobian(q_pos);

    return J.colPivHouseholderQr().solve(y_dot);
}

Eigen::Matrix<double, 5, 5> Controller::ComputeOutputJacobian(Eigen::Vector<double, N_Q> q_pos)
{
    Eigen::Matrix<double, 5, 5> J = Eigen::Matrix<double, 5, 5>::Zero();

    double s1 = sin(q_pos(0));
    double s12 = sin(q_pos(0) + q_pos(1));
    double s13 = sin(q_pos(0) + q_pos(1) + q_pos(2));
    double s14 = sin(q_pos(0) + q_pos(1) + q_pos(2) + q_pos(3));
    double s15 = sin(q_pos(0) + q_pos(1) + q_pos(2) + q_pos(3) + q_pos(4));

    double c1 = cos(q_pos(0));
    double c12 = cos(q_pos(0) + q_pos(1));
    double c13 = cos(q_pos(0) + q_pos(1) + q_pos(2));
    double c14 = cos(q_pos(0) + q_pos(1) + q_pos(2) + q_pos(3));
    double c15 = cos(q_pos(0) + q_pos(1) + q_pos(2) + q_pos(3) + q_pos(4));

    J(0, 0) = this->l_shin_ * c1 + this->l_thigh_ * c12;
    J(1, 0) = -this->l_shin_ * s1 - this->l_thigh_ * s12;
    J(2, 0) = 1.0;
    J(3, 0) = this->l_shin_ * c1 + this->l_thigh_ * c12 - this->l_thigh_ * c14 - this->l_shin_ * c15;
    J(4, 0) =-this->l_shin_ * s1 - this->l_thigh_ * s12 + this->l_thigh_ * s14 + this->l_shin_ * s15;

    J(0, 1) = this->l_thigh_ * c12;
    J(1, 1) = -this->l_thigh_ * s12;
    J(2, 1) = 1.0;
    J(3, 1) = this->l_thigh_ * c12 - this->l_thigh_ * c14 - this->l_shin_ * c15;
    J(4, 1) =-this->l_thigh_ * s12 + this->l_thigh_ * s14 + this->l_shin_ * s15;

    J(0, 2) = 0.0;
    J(1, 2) = 0.0;
    J(2, 2) = 1.0;
    J(3, 2) =-this->l_thigh_ * c14 - this->l_shin_ * c15;
    J(4, 2) = this->l_thigh_ * s14 + this->l_shin_ * s15;

    J(0, 3) = 0.0;
    J(1, 3) = 0.0;
    J(2, 3) = 0.0;
    J(3, 3) =-this->l_thigh_ * c14 - this->l_shin_ * c15;
    J(4, 3) = this->l_thigh_ * s14 + this->l_shin_ * s15;

    J(0, 4) = 0.0;
    J(1, 4) = 0.0;
    J(2, 4) = 0.0;
    J(3, 4) =-this->l_shin_ * c15;
    J(4, 4) = this->l_shin_ * s15;

    return J;
}

Eigen::Vector<double, N_OUTPUTS> Controller::CalculateOutputVel(Eigen::Vector<double, N_Q> q_pos, Eigen::Vector<double, N_Q> q_vel)
{
    Eigen::Matrix<double, 5, 5> J = ComputeOutputJacobian(q_pos);

    return J * q_vel;
}

Eigen::Vector<double, N_OUTPUTS> Controller::CalculateOutputsInStanceFootFrame(Eigen::Vector<double, N_Q> q_pos)
{
    // Get the joint position values
    double q_stf_ankle = q_pos(0);
    double q_stf_knee = q_pos(1);
    double q_stf_hip = q_pos(2);
    double q_swf_hip = q_pos(3);
    double q_swf_knee = q_pos(4); 

    // Compute the outputs

    // Compute the base position in the world frame
    // double p_x_com_stf_frame = this->l_shin_ * sin(q_stf_ankle) + this->l_thigh_ * sin(q_stf_ankle + q_stf_knee);
    // double p_z_com_stf_frame = this->l_shin_ * cos(q_stf_ankle) + this->l_thigh_ * cos(q_stf_ankle + q_stf_knee);
    double p_x_com_stf_frame = this->l_shin_ * sin(q_stf_ankle) + this->l_thigh_ * sin(q_stf_ankle + q_stf_knee);
    double p_z_com_stf_frame = this->l_shin_ * cos(q_stf_ankle) + this->l_thigh_ * cos(q_stf_ankle + q_stf_knee);

    // Compute the base pitch in the world frame
    double pitch_com_stf_frame = (q_stf_ankle + q_stf_knee) + q_stf_hip;

    // std::cout << "q_stf_ankle: " << q_stf_ankle << std::endl;
    // std::cout << "q_stf_knee: " << q_stf_knee << std::endl; 

    // Compute the swing foot position in the world frame
    double p_x_swf_stf_frame = p_x_com_stf_frame - this->l_thigh_ * sin(q_stf_ankle + q_stf_knee + q_stf_hip + q_swf_hip) - this->l_shin_ * sin(q_stf_ankle + q_stf_knee + q_stf_hip + q_swf_hip + q_swf_knee);
    double p_z_swf_stf_frame = p_z_com_stf_frame - this->l_thigh_ * cos(q_stf_ankle + q_stf_knee + q_stf_hip + q_swf_hip) - this->l_shin_ * cos(q_stf_ankle + q_stf_knee + q_stf_hip + q_swf_hip + q_swf_knee);



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
    y_world_frame(OutputIDX::COM_POS_X) = y_stf_frame(OutputIDX::COM_POS_X) + this->p_x_stf_world_frame_;
    y_world_frame(OutputIDX::COM_POS_Z) = y_stf_frame(OutputIDX::COM_POS_Z);
    y_world_frame(OutputIDX::COM_THETA) = y_stf_frame(OutputIDX::COM_THETA);
    y_world_frame(OutputIDX::SWF_POS_X) = y_stf_frame(OutputIDX::SWF_POS_X) + this->p_x_stf_world_frame_;
    y_world_frame(OutputIDX::SWF_POS_Z) = y_stf_frame(OutputIDX::SWF_POS_Z);

    // Return the outputs in the world frame
    return y_world_frame;
}

void Controller::UpdateController(Eigen::Vector<double, N_Q> q_pos, 
                                  Eigen::Vector<double, N_Q> q_vel,
                                  double t_step)
{
    // Calculate the outputs in the world frame

}

Eigen::Vector<double, 2> Controller::CalculatePeriod1ImpactRef(double vel_x_ref, double lambda, double T_SSP, double T_DSP)
{
    Eigen::Vector<double, 2> p1_impact_ref;

    // Calculate sigma 1 (Eq. 17)
    double sigma_1 = CalculateSigma1(lambda, T_SSP);

    // Calculate the desired pre-impact state (Eq. 20)
    p1_impact_ref(0) = vel_x_ref * (T_SSP + T_DSP) / (2.0 + T_DSP * sigma_1);
    p1_impact_ref(1) = sigma_1 * vel_x_ref * (T_SSP + T_DSP) / (2.0 + T_DSP * sigma_1);

    // Return the desired period 1 pre-impact state
    return p1_impact_ref;
}

double Controller::CalculateSigma1(double lambda, double T_SSP)
{
    return lambda * coth(T_SSP * lambda / 2.0);
}

Eigen::Vector<double, 2> Controller::CalculateDeadbeatGains(double T_SSP, double T_DSP, double lambda)
{
    Eigen::Vector<double, 2> K_deadbeat;

    K_deadbeat(0) = 1.0;
    K_deadbeat(1) = T_DSP + 1.0 / lambda * coth(T_SSP * lambda);

    return K_deadbeat;
}

Eigen::Vector<double, 2> Controller::CalculateSSPPreImpactState(double lambda, Eigen::Vector<double, 2> x_0, double t)
{
    Eigen::Matrix<double, 2, 2> V, V_inv, S;

    V << 1.0, 1.0,
         lambda, -lambda;

    V_inv = V.inverse();

    double exp_eig_1 = exp(lambda * t);
    double exp_eig_2 = exp(-lambda * t);

    S << exp_eig_1, 0,
         0, exp_eig_2;

    Eigen::Vector<double, 2> x_ssp_ref = V * S * V_inv * x_0;

    return x_ssp_ref;
}

double Controller::coth(double x)
{
    return (exp(x) + exp(-x)) / (exp(x) - exp(-x));
}

std::vector<double> Controller::CalculateSwfXCoeffs(double swf_pos_x_init, double swf_vel_x_init, 
                                                    double swf_pos_x_end, double swf_vel_x_end, 
                                                    double swf_pos_x_middle, double t_swf_pos_x_middle, 
                                                    double T_SSP) 
{
    Eigen::Matrix<double, 5, 5> A = Eigen::Matrix<double, 5, 5>::Zero();
    
    Eigen::Vector<double, 5> B = Eigen::Vector<double, 5>::Zero();

    A << 0, 0, 0, 0, 1,
         0, 0, 0, 1, 0,
         pow(T_SSP, 4), pow(T_SSP, 3), pow(T_SSP, 2), T_SSP, 1,
         4 * pow(T_SSP, 3), 3 * pow(T_SSP, 2), 2 * T_SSP, 1, 0,
         pow(t_swf_pos_x_middle, 4), pow(t_swf_pos_x_middle, 3), pow(t_swf_pos_x_middle, 2), t_swf_pos_x_middle, 1;

    B << swf_pos_x_init, swf_vel_x_init, swf_pos_x_end, swf_vel_x_end, swf_pos_x_middle;

    Eigen::Vector<double, 5> coeffs = A.colPivHouseholderQr().solve(B);

    return std::vector<double>(coeffs.data(), coeffs.data() + coeffs.size());
}

double Controller::GetSwfPosXRef(const std::vector<double>& coeffs, double t) 
{
    return coeffs[0] * pow(t, 4) + coeffs[1] * pow(t, 3) + coeffs[2] * pow(t, 2) + coeffs[3] * t + coeffs[4];
}

double Controller::GetSwfVelXRef(const std::vector<double>& coeffs, double t) 
{
    return 4.0 * coeffs[0] * pow(t, 3) + 3.0 * coeffs[1] * pow(t, 2) + 2.0 * coeffs[2] * t + coeffs[3];
}

std::vector<double> Controller::CalculateSwfZCoeffs(double x0, double v0, double xf, double vf, double x_max, double t_max, double T) 
{
    Eigen::Matrix<double, 5, 5> A = Eigen::Matrix<double, 5, 5>::Zero();
    
    Eigen::Vector<double, 5> B = Eigen::Vector<double, 5>::Zero();

    A << 0, 0, 0, 0, 1,
         0, 0, 0, 1, 0,
         pow(T, 4), pow(T, 3), pow(T, 2), T, 1,
         4 * pow(T, 3), 3 * pow(T, 2), 2 * T, 1, 0,
         pow(t_max, 4), pow(t_max, 3), pow(t_max, 2), t_max, 1;

    B << x0, v0, xf, vf, x_max;

    Eigen::Vector<double, 5> coeffs = A.colPivHouseholderQr().solve(B);

    return std::vector<double>(coeffs.data(), coeffs.data() + coeffs.size());
}

double Controller::GetSwfPosZRef(const std::vector<double>& coeffs, double t) 
{
    return coeffs[0] * pow(t, 4) + coeffs[1] * pow(t, 3) + coeffs[2] * pow(t, 2) + coeffs[3] * t + coeffs[4];
}

double Controller::GetSwfVelZRef(const std::vector<double>& coeffs, double t) 
{
    return 4.0 * coeffs[0] * pow(t, 3) + 3.0 * coeffs[1] * pow(t, 2) + 2.0 * coeffs[2] * t + coeffs[3];
}

Eigen::Vector<double, N_Q> Controller::ResetMapQ(Eigen::Vector<double, N_Q> q_pos)
{
    // Calculate the current outputs
    Eigen::Vector<double, N_OUTPUTS> y_pre_impact_world_frame = CalculateOutputsInWorldFrame(q_pos);

    std::cout << "y_pre_impact_world_frame: " << y_pre_impact_world_frame.transpose() << std::endl;

    // Update the stance foot world position
    double p_x_stf_pre_impact = this->p_x_stf_world_frame_;
    double p_x_stf_post_impact = y_pre_impact_world_frame(OutputIDX::SWF_POS_X);

    // Calculate the outputs after the reset map
    Eigen::Vector<double, N_OUTPUTS> y_post_impact_world_frame;

    y_post_impact_world_frame(OutputIDX::COM_POS_X) = y_pre_impact_world_frame(OutputIDX::COM_POS_X);
    y_post_impact_world_frame(OutputIDX::COM_POS_Z) = y_pre_impact_world_frame(OutputIDX::COM_POS_Z);
    y_post_impact_world_frame(OutputIDX::COM_THETA) = y_pre_impact_world_frame(OutputIDX::COM_THETA);
    y_post_impact_world_frame(OutputIDX::SWF_POS_X) = p_x_stf_pre_impact;
    y_post_impact_world_frame(OutputIDX::SWF_POS_Z) = 0.0;

    std::cout << "y_post_impact_world_frame: " << y_post_impact_world_frame.transpose() << std::endl;

    // Calculate the post impact outputs in the stance foot frame
    Eigen::Vector<double, N_OUTPUTS> y_post_impact_stf_frame;
    y_post_impact_stf_frame(OutputIDX::COM_POS_X) = y_post_impact_world_frame(OutputIDX::COM_POS_X) - p_x_stf_post_impact;
    y_post_impact_stf_frame(OutputIDX::COM_POS_Z) = y_post_impact_world_frame(OutputIDX::COM_POS_Z);
    y_post_impact_stf_frame(OutputIDX::COM_THETA) = y_post_impact_world_frame(OutputIDX::COM_THETA);
    y_post_impact_stf_frame(OutputIDX::SWF_POS_X) = y_post_impact_world_frame(OutputIDX::SWF_POS_X) - p_x_stf_post_impact;
    y_post_impact_stf_frame(OutputIDX::SWF_POS_Z) = y_post_impact_world_frame(OutputIDX::SWF_POS_Z);

    std::cout << "y_post_impact_stf_frame: " << y_post_impact_stf_frame.transpose() << std::endl;

    // Calculate the inverse kinematics to get the joint angles
    Eigen::Vector<double, N_Q> q_pos_post_impact = SolveIK(y_post_impact_stf_frame);

    // Set the stf world position to the post impact value
    this->p_x_stf_world_frame_ = p_x_stf_post_impact;

    return q_pos_post_impact;
}

void Controller::ComputeResetMap(Eigen::Vector<double, N_Q> q_pos_pre_impact, Eigen::Vector<double, N_Q> q_vel_pre_impact,
                                    Eigen::Vector<double, N_Q> &q_pos_post_impact, Eigen::Vector<double, N_Q> &q_vel_post_impact)
{
    // Calculate the current output velocities
    Eigen::Vector<double, N_OUTPUTS> y_dot_pre_impact_world_frame = CalculateOutputVel(q_pos_pre_impact, q_vel_pre_impact);

    // Get the joint coordinates after impact
    q_pos_post_impact = ResetMapQ(q_pos_pre_impact);

    // Calculate the state velocity after impact
    q_vel_post_impact = SolveIKDerivative(y_dot_pre_impact_world_frame, q_pos_post_impact);
}

Eigen::Vector<double, 3> Controller::GetStfPosWorldFrame()
{
    return Eigen::Vector<double, 3>(this->p_x_stf_world_frame_, 0.0, 0.0);
}