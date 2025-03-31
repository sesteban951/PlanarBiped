
#include "controller.h"
#include "simulator.h"
#include "logger.h"
#include "config.h" 

#include "stdio.h"
#include <iostream>
#include <iomanip>

#include <chrono>

int main() 
{
    // Set the printing precision
    std::cout << std::fixed;
    std::cout << std::setprecision(3);

    Controller controller;
    Simulator simulator;

    Logger logger("/home/adrian/PlanarBiped/cpp/logs/log.csv");

    std::string log_labels = "t, q_1, q_2, q_3, q_4, q_5, q_1_dot, q_2_dot, q_3_dot, q_4_dot, q_5_dot, eta_1, eta_2, eta_3, eta_4, eta_1_dot, eta_2_dot, eta_3_dot, eta_4_dot, z_1, z_1_dot,";

    logger.AddLabels(log_labels);

    simulator.Initialize("/home/adrian/PlanarBiped/models/biped/biped_pinned_hotdog.xml");

    double simulation_rate = 500.0;
    double visualization_rate = 60.0;

    // Set the initial conditions for the simulation
    Eigen::Vector<double, N_OUTPUTS> y_ref;
    y_ref(OutputIDX::COM_POS_X) = 0.0;
    y_ref(OutputIDX::COM_POS_Z) = controller.GetBasePosZRef();
    y_ref(OutputIDX::COM_THETA) = 0.0;
    y_ref(OutputIDX::SWF_POS_X) = 0.0;
    y_ref(OutputIDX::SWF_POS_Z) = 0.0;

    // Solve for the joint angles using the controller
    Eigen::Vector<double, N_Q> q_pos_initial = controller.SolveIK(y_ref);

    // Eigen::Vector<double, N_Q> q_pos_pre_impact = q_pos_initial;
    // Eigen::Vector<double, N_Q> q_vel_pre_impact(0.20, 0.10, 0.0, 0.10, 0.20);
    // Eigen::Vector<double, N_OUTPUTS> y_pre_impact_world_frame = controller.CalculateOutputsInWorldFrame(q_pos_pre_impact);
    // Eigen::Vector<double, N_OUTPUTS> y_dot_pre_impact_world_frame = controller.CalculateOutputVel(q_pos_pre_impact, q_vel_pre_impact);
    // Eigen::Vector<double, N_Q> q_pos_post_impact;
    // Eigen::Vector<double, N_Q> q_vel_post_impact;
    // controller.ComputeResetMap(q_pos_pre_impact, q_vel_pre_impact, q_pos_post_impact, q_vel_post_impact);
    // Eigen::Vector<double, N_OUTPUTS> y_post_impact_world_frame = controller.CalculateOutputsInWorldFrame(q_pos_post_impact);
    // Eigen::Vector<double, N_OUTPUTS> y_dot_post_impact_world_frame = controller.CalculateOutputVel(q_pos_post_impact, q_vel_post_impact);
    // std::cout << "q_pos_pre_impact: " << q_pos_pre_impact.transpose() << std::endl;
    // std::cout << "q_vel_pre_impact: " << q_vel_pre_impact.transpose() << std::endl;
    // std::cout << "q_pos_post_impact: " << q_pos_post_impact.transpose() << std::endl;
    // std::cout << "q_vel_post_impact: " << q_vel_post_impact.transpose() << std::endl;
    // std::cout << "y_pre_impact_world_frame: " << y_pre_impact_world_frame.transpose() << std::endl;
    // std::cout << "y_post_impact_world_frame: " << y_post_impact_world_frame.transpose() << std::endl;
    // std::cout << "y_dot_pre_impact_world_frame: " << y_dot_pre_impact_world_frame.transpose() << std::endl;
    // std::cout << "y_dot_post_impact_world_frame: " << y_dot_post_impact_world_frame.transpose() << std::endl;
    // exit(0);

    simulator.SetState(q_pos_initial);

    // Simulation loop
    auto last_sim_time = std::chrono::high_resolution_clock::now();
    auto last_vis_time = std::chrono::high_resolution_clock::now();
    auto program_start_time = std::chrono::high_resolution_clock::now();

    Eigen::Vector<double, N_Q> q_pos_ref = simulator.GetGeneralizedPosition();
    Eigen::Vector<double, N_Q> q_vel_ref = simulator.GetGeneralizedVelocity();
    Eigen::Vector<double, N_Q> q_tor_ref = Eigen::Vector<double, N_Q>::Zero();

    double alpha = 0.999;
    double com_vel_x_filtered = 0.0;

    while (true)
    {
        // Get current time
        auto now = std::chrono::high_resolution_clock::now();

        // Calculate elapsed time since last simulation step
        std::chrono::duration<double> elapsed_sim_time = now - last_sim_time;
        
        // Run the simulation step if enough time has passed
        if (elapsed_sim_time.count() >= 1.0 / simulation_rate)
        {
            // Get the mujoco simulation time
            double t_curr = MJ_DATA_PTR->time;

            // Print the time
            std::cout << "t: " << t_curr << "\t";

            std::cout << std::endl;

            // Get the latest state
            Eigen::Vector<double, N_Q> q_pos = simulator.GetGeneralizedPosition();
            Eigen::Vector<double, N_Q> q_vel = simulator.GetGeneralizedVelocity();

            controller.SetMassMatrix(simulator.GetMassMatrix());

            // Calculate outputs in the world frame
            Eigen::Vector<double, N_OUTPUTS> y_world_frame = controller.CalculateOutputsInWorldFrame(q_pos);
            Eigen::Vector<double, N_OUTPUTS> y_dot_world_frame = controller.CalculateOutputVel(q_pos, q_vel);
            
            Eigen::Vector<double, 3> base_pos_world = simulator.GetTorsoPos();
            Eigen::Vector<double, 3> base_vel_world = simulator.GetTorsoVel();
            Eigen::Vector<double, 2> stf_pos_world = simulator.GetStfPos();
            Eigen::Vector<double, 2> stf_vel_world = simulator.GetStfVel();
            Eigen::Vector<double, 2> swf_pos_world = simulator.GetSwfPos();
            Eigen::Vector<double, 2> swf_vel_world = simulator.GetSwfVel();

            std::cout << "com_pos_x: " << y_world_frame(OutputIDX::COM_POS_X) << "\t";
            std::cout << "com_pos_z: " << y_world_frame(OutputIDX::COM_POS_Z) << "\t";
            std::cout << "com_theta: " << y_world_frame(OutputIDX::COM_THETA) << "\t";
            std::cout << "swf_pos_x: " << y_world_frame(OutputIDX::SWF_POS_X) << "\t";
            std::cout << "swf_pos_z: " << y_world_frame(OutputIDX::SWF_POS_Z) << "\t";
            std::cout << "com_vel_x: " << y_dot_world_frame(OutputIDX::COM_POS_X) << "\t";
            std::cout << "com_vel_z: " << y_dot_world_frame(OutputIDX::COM_POS_Z) << "\t";
            std::cout << "com_ang_vel: " << y_dot_world_frame(OutputIDX::COM_THETA) << "\t";
            std::cout << "swf_vel_x: " << y_dot_world_frame(OutputIDX::SWF_POS_X) << "\t";
            std::cout << "swf_vel_z: " << y_dot_world_frame(OutputIDX::SWF_POS_Z) << "\t";
            std::cout << std::endl;

            std::cout << "com_pos_x: " << base_pos_world(0) << "\t";
            std::cout << "com_pos_z: " << base_pos_world(1) << "\t";
            std::cout << "com_theta: " << base_pos_world(2) << "\t";
            std::cout << "swf_pos_x: " << swf_pos_world(0) << "\t";
            std::cout << "swf_pos_z: " << swf_pos_world(1) << "\t";
            std::cout << "com_vel_x: " << base_vel_world(0) << "\t";
            std::cout << "com_vel_z: " << base_vel_world(1) << "\t";
            std::cout << "com_ang_vel: " << base_vel_world(2) << "\t";
            std::cout << "swf_vel_x: " << swf_vel_world(0) << "\t";
            std::cout << "swf_vel_z: " << swf_vel_world(1) << "\t";
            std::cout << std::endl;

            com_vel_x_filtered = alpha * com_vel_x_filtered + (1.0 - alpha) * y_dot_world_frame(OutputIDX::COM_POS_X);
            std::cout << "com_vel_x_filtered: " << com_vel_x_filtered << std::endl;

            // Check for step update + reset map
            bool update_stance_foot = controller.CheckForStanceFootUpdate(t_curr, q_pos, q_vel);

            if(update_stance_foot == true)
            {
                // Compute the reset map
                Eigen::Vector<double, N_Q> q_pos_pre_impact = q_pos;
                Eigen::Vector<double, N_Q> q_vel_pre_impact = q_vel;
                Eigen::Vector<double, N_Q> q_pos_post_impact;
                Eigen::Vector<double, N_Q> q_vel_post_impact;
                controller.ComputeResetMap(q_pos_pre_impact, q_vel_pre_impact, q_pos_post_impact, q_vel_post_impact);

                // Set the new state
                //simulator.SetState(q_pos_post_impact);
                simulator.SetState(q_pos_post_impact, q_vel_post_impact);

                // Get the stance foot position in the world frame
                Eigen::Vector<double, 3> stf_pos_world_frame = controller.GetStfPosWorldFrame();

                // Update the stance foot position in the simulator
                simulator.UpdateStanceFootPosition(stf_pos_world_frame);

                if(t_curr < 3.0)
                {
                    controller.SetVelRef(0.0);
                }
                else if(t_curr < 6.0)
                {
                    controller.SetVelRef(0.4);
                }
                else
                {
                    controller.SetVelRef(0.75);
                }
            }

            // Update controller
            controller.UpdateController(q_pos, q_vel, t_curr, q_pos_ref, q_vel_ref, q_tor_ref);

            // std::cout << "q_pos: " << q_pos.transpose() << std::endl;
            // std::cout << "q_pos_ref: " << q_pos_ref.transpose() << std::endl;
            // std::cout << "q_vel: " << q_vel.transpose() << std::endl;
            // std::cout << "q_vel_ref: " << q_vel_ref.transpose() << std::endl;

            // Calculate the motor torques
            Eigen::Vector<double, 4> motor_torques = controller.CalculateMotorTorques(q_pos, q_vel, q_pos_ref, q_vel_ref, q_tor_ref);

            // Print the motor torques
            // std::cout << "Motor torques: " << motor_torques.transpose() << std::endl;

            // Set the motor torques in the simulator
            simulator.SetMotorTorques(motor_torques);


            // Logging
            Eigen::Matrix<double, 5, 4> B = Eigen::Matrix<double, 5, 4>::Zero();
            B.block<4, 4>(1, 0) = Eigen::Matrix<double, 4, 4>::Identity();

            Eigen::Vector<double, 8> eta;
            eta.segment<4>(0) = B.transpose() * q_pos;
            eta.segment<4>(4) = B.transpose() * q_vel;

            Eigen::Matrix<double, 1, 5> N = Eigen::Matrix<double, 1, 5>::Zero();
            N(0, 0) = 1.0;

            Eigen::Matrix<double, N_Q, N_Q> M = simulator.GetMassMatrix();

            Eigen::Vector<double, 2> z;
            z(0) = N * q_pos;
            z(1) = N * M * q_vel;

            Eigen::Vector<double, Eigen::Dynamic> log_data(21);
            log_data << t_curr, q_pos, q_vel, eta, z;

            logger.WriteToLog(log_data);



            // Step the Mujoco simulation
            simulator.PropagateDynamics();

            // Update the last simulation time
            last_sim_time = now;

            std::cout << std::endl;
        }

        // Calculate elapsed time since last visualization update
        std::chrono::duration<double> elapsed_vis_time = now - last_vis_time;

        // Update visualization if enough time has passed
        if (elapsed_vis_time.count() >= 1.0 / visualization_rate)
        {
            simulator.UpdateScene();
            last_vis_time = now;
        }
    }

    

    return 0;
}
