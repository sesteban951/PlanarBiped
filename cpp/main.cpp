
#include "controller.h"
#include "simulator.h"
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

    simulator.Initialize("/home/adrian/PlanarBiped/models/biped/biped_pinned.xml");

    double simulation_rate = 500.0;
    double visualization_rate = 60000.0;

    // Set the initial conditions for the simulation
    Eigen::Vector<double, N_OUTPUTS> y_ref;
    y_ref(OutputIDX::COM_POS_X) = 0.0;
    y_ref(OutputIDX::COM_POS_Z) = 0.95;
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

            // Get the latest state
            Eigen::Vector<double, N_Q> q_pos = simulator.GetGeneralizedPosition();
            Eigen::Vector<double, N_Q> q_vel = simulator.GetGeneralizedVelocity();

            // Check for step update + reset map
            

            // Update controller
            controller.UpdateController(q_pos, q_vel, t_curr, q_pos_ref, q_vel_ref, q_tor_ref);

            std::cout << "q_pos: " << q_pos.transpose() << std::endl;
            std::cout << "q_pos_ref: " << q_pos_ref.transpose() << std::endl;
            std::cout << "q_vel: " << q_vel.transpose() << std::endl;
            std::cout << "q_vel_ref: " << q_vel_ref.transpose() << std::endl;

            // Calculate the motor torques
            Eigen::Vector<double, 4> motor_torques = controller.CalculateMotorTorques(q_pos, q_vel, q_pos_ref, q_vel_ref, q_tor_ref);

            // Print the motor torques
            std::cout << "Motor torques: " << motor_torques.transpose() << std::endl;

            // Set the motor torques in the simulator
            simulator.SetMotorTorques(motor_torques);

            // if(std::abs(rem) < 0.001)
            // {
            //     std::cout << "Resetting map" << std::endl;
            //     q_pos = controller.ResetMapQ(q_pos);

            //     // Get the stance foot position in the world frame
            //     Eigen::Vector<double, 3> stf_pos_world_frame = controller.GetStfPosWorldFrame();

            //     // Update the stance foot position in the simulator
            //     simulator.UpdateStanceFootPosition(stf_pos_world_frame);

            //     std::cout << "Stance foot position: " << stf_pos_world_frame.transpose() << std::endl;

            //     //exit(0);
            // }
            // Eigen::Vector<double, 3> stf_pos_world_frame(t_curr / 5.0, 0.0, 0.0);
            // //simulator.UpdateStanceFootPosition(stf_pos_world_frame);

            // // Calculate the outputs in the stf frame
            // Eigen::Vector<double, N_OUTPUTS> y_stf_frame = controller.CalculateOutputsInStanceFootFrame(q_pos);

            // std::cout << "y_pos: " << y_stf_frame.transpose() << std::endl;

            // // Set the joint positions in the Mujoco data structure
            // simulator.SetState(q_pos);

            // Step the Mujoco simulation
            mj_step(MJ_MODEL_PTR, MJ_DATA_PTR);

            // Update the last simulation time
            last_sim_time = now;
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
