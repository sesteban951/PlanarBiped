%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lower to higher dimensional integrator (Trajectory tracking)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% parameters
params.k_single = 5.0;   % single integrator gain
params.kp_double = 5.0;  % double integrator proportional gain
params.kd_double = 2.0;  % double integrator derivative gain

params.A = 0.5;        % desired traj amplitude
params.offset = 0.0;   % desired traj offset
params.T = 5.0;        % desired traj period

% initial conditions
z0_single =  1;  % p
x0_double = [1;  % p 
             0]; % v
x0 = [z0_single; x0_double];

% simulation 
t_sim = 15.0;
dt = 0.01;
t = 0:dt:t_sim;

% initialize the state
[t, x] = ode45(@(t, x) dynamics(t, x, params), t, x0);

% extract the states
z = x(:, 1);
x = x(:, 2:3);

% plot the results
figure;

% plot the tracking
subplot(1,2,1);
hold on; grid on;

% plot the desired trajectory
t_traj = 0:0.01:t_sim;
p_des = params.A * sin(2 * pi * t_traj / params.T) + params.offset;
plot(t_traj, p_des, 'k--', 'LineWidth', 1.5);

% plot the solutions
plot(t, z, 'b', 'LineWidth', 1.5);      % single integrator
plot(t, x(:,1), 'r', 'LineWidth', 1.5); % double integrator
title('Tracking');
legend('', 'Single', 'Double');

% plot double integrator traj
subplot(1,2,2);
hold on; grid on; axis equal;
xline(0); yline(0);
plot(x(:,1), x(:,2), 'r', 'LineWidth', 1.5);
xlabel('p');
ylabel('v');
title('Double Integrator Trajectory');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% function that does both the double and single integrator
function xdot = dynamics(t, x, params)

    % unpack the state
    p_single = x(1);
    z = p_single;

    p_double = x(2);
    v_double = x(3);
    x = [p_double;
         v_double];

    % get the current desired trajectory
    p_des = params.A * sin(2 * pi * t / params.T) + params.offset;

    % compute the control input for the single integrator
    v = -params.k_single * (z - p_des);  

    % single integrator dynamics
    zdot = v;

    % compute the control input for the double integrator
    pdot_des = (2 * pi / params.T) * (params.A) * cos(2 * pi * t / params.T);
    zddot = -params.k_single * (v - pdot_des);
    y = x(1) - z;
    ydot = x(2) - zdot;
    u = zddot - params.kp_double * y - params.kd_double * ydot;

    % double integrator dynamics
    x = [p_double;
         v_double];
    A_double = [0, 1;
                0, 0];
    B_double = [0;
                1];
    xdot = A_double * x + B_double * u;

    xdot = [zdot; xdot];
end

