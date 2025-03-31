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
                                    double t_step,
                                    Eigen::Vector<double, N_Q> &q_pos_ref,
                                    Eigen::Vector<double, N_Q> &q_vel_ref,
                                    Eigen::Vector<double, N_Q> &q_tor_ref);

    public: Eigen::Vector<double, 4> CalculateMotorTorques(Eigen::Vector<double, N_Q> q_pos, 
                                                            Eigen::Vector<double, N_Q> q_vel,
                                                            Eigen::Vector<double, N_Q> q_pos_ref, 
                                                            Eigen::Vector<double, N_Q> q_vel_ref,
                                                            Eigen::Vector<double, N_Q> q_tor_ref);

    public: Eigen::Vector<double, N_Q> ResetMapQ(Eigen::Vector<double, N_Q> q_pos);

    public: void ComputeResetMapSimple(Eigen::Vector<double, N_Q> q_pos_pre_impact, Eigen::Vector<double, N_Q> q_vel_pre_impact,
                                        Eigen::Vector<double, N_Q> &q_pos_post_impact, Eigen::Vector<double, N_Q> &q_vel_post_impact);

    public: void ComputeResetMap(Eigen::Vector<double, N_Q> q_pos_pre_impact, Eigen::Vector<double, N_Q> q_vel_pre_impact,
                                    Eigen::Vector<double, N_Q> &q_pos_post_impact, Eigen::Vector<double, N_Q> &q_vel_post_impact);


    public: Eigen::Vector<double, 3> GetStfPosWorldFrame();

    bool CheckForStanceFootUpdate(double t_curr, Eigen::Vector<double, N_Q> q_pos, Eigen::Vector<double, N_Q> q_vel);

    public: double GetBasePosZRef(){return com_pos_z_ref_;}

    public: void SetVelRef(double v_x_ref){v_x_ref_ = v_x_ref;}

    private: Eigen::Matrix<double, N_Q, N_Q> M_ = Eigen::Matrix<double, N_Q, N_Q>::Identity();

    public: void SetMassMatrix(Eigen::Matrix<double, N_Q, N_Q> M){M_ = M;}

    // Model parameters
    private: double l_thigh_ = 0.5;
    private: double l_shin_ = 0.5;

    // Control parameters
    private: double com_pos_z_ref_ = 0.9;
    private: double com_theta_ref_ = 0.0;

    private: double T_SSP_ = 0.3;
    private: double T_DSP_ = 0.0;
    private: double g_ = 9.81;

    private: double kp_hip_ = 400.0;
    private: double kp_knee_ = 400.0;
    
    private: double kd_hip_ = 20.0;
    private: double kd_knee_ = 20.0;

    private: double t_step_start_ = 0.0;


    private: double p_x_stf_world_frame_ = 0.0;

    private: double v_x_ref_ = 0.0;

    private: double swf_pos_x_init_ = 0.0;
    private: double swf_vel_x_init_ = 0.0;
    private: double swf_pos_x_end_ = 0.0;
    private: double swf_vel_x_end_ = 0.0;
    private: double swf_pos_x_middle_ = 0.0;
    private: double t_swf_pos_x_middle_ = 0.0;

    private: double swf_pos_z_init_ = 0.0;
    private: double swf_vel_z_init_ = 0.0;
    private: double swf_pos_z_end_ = 0.0;
    private: double swf_vel_z_end_ = 0.0;
    private: double swf_pos_z_middle_ = 0.0;
    private: double t_swf_pos_z_middle_ = 0.0;

};

#endif // CONTROLLER_H