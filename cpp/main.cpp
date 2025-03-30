
#include "controller.h"
#include "simulator.h"
#include "config.h" 

#include <chrono>

int main() 
{
    Controller controller;
    Simulator simulator;

    simulator.Initialize("/home/adrian/PlanarBiped/models/biped/biped_pinned.xml");

    double simulation_rate = 500.0;
    double visualization_rate = 60.0;

    // Set the initial conditions for the simulation
    Eigen::Vector<double, N_OUTPUTS> y_ref;
    y_ref(OutputIDX::COM_POS_X) = 0.0;
    y_ref(OutputIDX::COM_POS_Z) = 0.9;
    y_ref(OutputIDX::COM_THETA) = 0.0;
    y_ref(OutputIDX::SWF_POS_X) = 0.3;
    y_ref(OutputIDX::SWF_POS_Z) = 0.2;

    // Solve for the joint angles using the controller
    Eigen::Vector<double, N_Q> q_pos_initial = controller.SolveIK(y_ref);

    simulator.SetState(q_pos_initial);

    // Simulation loop
    auto last_sim_time = std::chrono::high_resolution_clock::now();
    auto last_vis_time = std::chrono::high_resolution_clock::now();

    while (true)
    {
        // Get current time
        auto now = std::chrono::high_resolution_clock::now();

        // Calculate elapsed time since last simulation step
        std::chrono::duration<double> elapsed_sim_time = now - last_sim_time;
        
        // Run the simulation step if enough time has passed
        if (elapsed_sim_time.count() >= 1.0 / simulation_rate)
        {
            // Set the outputs
            Eigen::Vector<double, N_OUTPUTS> y_ref;
            y_ref(OutputIDX::COM_POS_X) = 0.0;
            y_ref(OutputIDX::COM_POS_Z) = 0.9;
            y_ref(OutputIDX::COM_THETA) = 0.0;
            y_ref(OutputIDX::SWF_POS_X) = 0.3;
            y_ref(OutputIDX::SWF_POS_Z) = 0.2;

            // Solve for the joint angles using the controller
            Eigen::Vector<double, N_Q> q_pos = controller.SolveIK(y_ref);

            for(int i = 0; i < 4; i++)
            {
                MJ_DATA_PTR->qpos[i] = q_pos(i);
                MJ_DATA_PTR->qvel[i] = 0;
            }

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
