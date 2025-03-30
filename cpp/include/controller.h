#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "config.h"

#include <cmath> 

// Include eigen
#include <Eigen/Dense>

class Controller 
{
    public: Controller();

    public: Eigen::Vector<double, N_Q> SolveIK(Eigen::Vector<double, N_OUTPUTS> y_ref);

    public: Eigen::Vector<double, N_Q> SolveIKDerivative(Eigen::Vector<double, N_OUTPUTS> y_dot, Eigen::Vector<double, N_Q> q_pos);

    public: Eigen::Matrix<double, 5, 5> ComputeOutputJacobian(Eigen::Vector<double, N_Q> q_pos);

    public: Eigen::Vector<double, N_Q> CalculateOutputVel(Eigen::Vector<double, N_Q> q_pos, Eigen::Vector<double, N_Q> q_vel);

    public: Eigen::Vector<double, N_OUTPUTS> CalculateOutputsInStanceFootFrame(Eigen::Vector<double, N_Q> q_pos);

    public: Eigen::Vector<double, N_OUTPUTS> CalculateOutputsInWorldFrame(Eigen::Vector<double, N_Q> q_pos);

    public: Eigen::Vector<double, 2> CalculatePeriod1ImpactRef(double vel_x_ref, double lambda, double T_SSP, double T_DSP);

    public: double CalculateSigma1(double lambda, double T_SSP);

    public: Eigen::Vector<double, 2> CalculateDeadbeatGains(double T_SSP, double T_DSP, double lambda);

    public: Eigen::Vector<double, 2> CalculateSSPPreImpactState(double lambda, Eigen::Vector<double, 2> x_0, double t);

    public: double coth(double x);

    public: std::vector<double> CalculateSwfXCoeffs(double swf_pos_x_init, double swf_vel_x_init, 
                                                    double swf_pos_x_end, double swf_vel_x_end, 
                                                    double swf_pos_x_middle, double t_swf_pos_x_middle, 
                                                    double T_SSP);

    public: double GetSwfPosXRef(const std::vector<double>& coeffs, double t);

    public: double GetSwfVelXRef(const std::vector<double>& coeffs, double t);

    public: std::vector<double> CalculateSwfZCoeffs(double x0, double v0, double xf, double vf, double x_max, double t_max, double T);

    public: double GetSwfPosZRef(const std::vector<double>& coeffs, double t);

    public: double GetSwfVelZRef(const std::vector<double>& coeffs, double t);

    public: void UpdateController(Eigen::Vector<double, N_Q> q_pos, 
                            Eigen::Vector<double, N_Q> q_vel,
                            double t_step);

    public: Eigen::Vector<double, N_Q> ResetMapQ(Eigen::Vector<double, N_Q> q_pos);

    public: void ComputeResetMap(Eigen::Vector<double, N_Q> q_pos_pre_impact, Eigen::Vector<double, N_Q> q_vel_pre_impact,
                                    Eigen::Vector<double, N_Q> &q_pos_post_impact, Eigen::Vector<double, N_Q> &q_vel_post_impact);

    public: Eigen::Vector<double, 3> GetStfPosWorldFrame();

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