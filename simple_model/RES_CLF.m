%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RES-CLF example
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% system dynamics
params.A = [zeros(2,2), eye(2); 
            zeros(2,2), zeros(2,2)];
params.B = [zeros(2,2); eye(2)];

% gains
params.Kp = 2* diag([1.5, 1.25]);
params.Kd = 2* diag([0.5, 0.25]);

% scaling factor
params.epsilon = 0.2;

% desired state
params.x_des = [0; 0; 0; 0];

% initial condition
x0 = [0.25; 0.5; 0.5; 0.5];

% simulaiton
t_sim = 7.0;
dt = 0.01;
t = 0:dt:t_sim;

% simulate
use_epsilon = false;
[t, x] = ode45(@(t,x) dynamics(t, x, params, use_epsilon), t, x0);

use_epsilon = true;
[t_eps, x_eps] = ode45(@(t,x) dynamics(t, x, params, use_epsilon), t, x0);

subplot(2,2,[1, 3])
hold on; grid on;
plot(x(:,1), x(:,2), 'b', 'LineWidth', 2);
plot(x_eps(:,1), x_eps(:,2), 'r', 'LineWidth', 2);
xline(0);
yline(0);

subplot(2,2,2)
hold on; grid on;
plot(t, x(:,1), 'b', 'LineWidth', 2);
plot(t_eps, x_eps(:,1), 'r', 'LineWidth', 2);
yline(0);

subplot(2,2,4)
hold on; grid on;
plot(t, x(:,2), 'b', 'LineWidth', 2);
plot(t_eps, x_eps(:,2), 'r', 'LineWidth', 2);
yline(0);

% get closed loop damping ratio
Kp = params.Kp;
Kd = params.Kd;
A_cl = [zeros(2,2), eye(2); 
        -Kp,       -Kd];
damping_ratios = compute_damping_ratios(A_cl);

epsilon = params.epsilon;
A_cl_eps = [zeros(2,2),     eye(2); 
            -Kp/epsilon^2, -Kd/epsilon];
damping_ratios_eps = compute_damping_ratios(A_cl_eps);

disp('Damping Ratios')
disp('Without epsilon')
disp(damping_ratios)
disp('With epsilon')
disp(damping_ratios_eps)

function xdot = dynamics(t, x, params, use_epsilon)

    % extract outputs
    y = [x(1); x(2)];
    ydot = [x(3); x(4)];

    % extract desired outputs
    y_des = [params.x_des(1); params.x_des(2)];
    ydot_des = [params.x_des(3); params.x_des(4)];

    Kp = params.Kp;
    Kd = params.Kd;

    % compute the control input
    if use_epsilon == true
        % compute the control input
        epsilon = params.epsilon;
        u  = -(1/(epsilon^2)) * Kp * (y - y_des) - (1 / epsilon) * Kd * (ydot - ydot_des);
    else
        % compute the control input
        u  = -Kp * (y - y_des) - Kd * (ydot - ydot_des);
    end

    % system dynamics
    xdot = params.A*x + params.B*u
end

function damping_ratios = compute_damping_ratios(A_cl)
    % Compute eigenvalues
    eigenvalues = eig(A_cl);

    % Compute damping ratio for each complex eigenvalue
    damping_ratios = -real(eigenvalues) ./ abs(eigenvalues);
end