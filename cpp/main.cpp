
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
    double visualization_rate = 60.0;

    // Set the initial conditions for the simulation
    Eigen::Vector<double, N_OUTPUTS> y_ref;
    y_ref(OutputIDX::COM_POS_X) = 0.1;
    y_ref(OutputIDX::COM_POS_Z) = 0.8;
    y_ref(OutputIDX::COM_THETA) = 0.0;
    y_ref(OutputIDX::SWF_POS_X) = 0.4;
    y_ref(OutputIDX::SWF_POS_Z) = 0.0;

    // Solve for the joint angles using the controller
    Eigen::Vector<double, N_Q> q_pos_initial = controller.SolveIK(y_ref);

    Eigen::Vector<double, N_Q> q_pos = q_pos_initial;

    simulator.SetState(q_pos_initial);

    // Simulation loop
    auto last_sim_time = std::chrono::high_resolution_clock::now();
    auto last_vis_time = std::chrono::high_resolution_clock::now();
    auto program_start_time = std::chrono::high_resolution_clock::now();

    while (true)
    {
        // Get current time
        auto now = std::chrono::high_resolution_clock::now();

        // Calculate elapsed time since last simulation step
        std::chrono::duration<double> elapsed_sim_time = now - last_sim_time;
        
        // Run the simulation step if enough time has passed
        if (elapsed_sim_time.count() >= 1.0 / simulation_rate)
        {
            // Get the time since the program started in seconds
            double t_curr = std::chrono::duration_cast<std::chrono::duration<double>>(now - program_start_time).count();

            double rem = remainder(t_curr, 5.0);

            // Print the time
            std::cout << "t: " << t_curr << "\t" << "rem: " << rem << std::endl;


            if(std::abs(rem) < 0.001)
            {
                std::cout << "Resetting map" << std::endl;
                q_pos = controller.ResetMapQ(q_pos);

                // Get the stance foot position in the world frame
                Eigen::Vector<double, 3> stf_pos_world_frame = controller.GetStfPosWorldFrame();

                // Update the stance foot position in the simulator
                simulator.UpdateStanceFootPosition(stf_pos_world_frame);

                std::cout << "Stance foot position: " << stf_pos_world_frame.transpose() << std::endl;

                //exit(0);
            }
            Eigen::Vector<double, 3> stf_pos_world_frame(t_curr / 5.0, 0.0, 0.0);
            //simulator.UpdateStanceFootPosition(stf_pos_world_frame);

            // Calculate the outputs in the stf frame
            Eigen::Vector<double, N_OUTPUTS> y_stf_frame = controller.CalculateOutputsInStanceFootFrame(q_pos);

            std::cout << "y_pos: " << y_stf_frame.transpose() << std::endl;

            // Set the joint positions in the Mujoco data structure
            simulator.SetState(q_pos);

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
