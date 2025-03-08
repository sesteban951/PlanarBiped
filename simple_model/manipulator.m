%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Manipulator Example
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% manipulator parameters
params.m1 = 1.0;  % mass of link 1
params.m2 = 1.0;  % mass of link 2
params.l1 = 1.0;  % length of link 1
params.l2 = 1.0;  % length of link 2
params.b = 1.0;   % damping
params.g = 9.81; % gravity

% initial conditions
x0 = [0.1;  % q1
      0.1;  % q2
      0.0;  % q1dot
      0.0]; % q2dot

% simulation
t_sim = 10.0;
dt = 0.01;
tspan = 0:dt:t_sim;

% simualte the dynamics
[t, x] = ode45(@(t, x) dynamics(t, x, params), tspan, x0);

% extract the states
q = x(:, 1:2);
qdot = x(:, 3:4);

figure;
subplot(1,2,1);
hold on; grid on;
plot(t, q(:,1), 'b', 'LineWidth', 1.5);
plot(t, q(:,2), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
legend('q1', 'q2');
title('Joint Angles');

subplot(1,2,2);
hold on; grid on;
plot(t, qdot(:,1), 'b', 'LineWidth', 1.5);
plot(t, qdot(:,2), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Velocities (rad/s)');
legend('q1dot', 'q2dot');
title('Joint Velocities');

% manipulator dynamics
function xdot = dynamics(t, x, params)

    % unpack the state
    q    = [x(1); x(2)];  % joint angles
    qdot = [x(3); x(4)]; % joint velocities

    % get the manipulator equations
    [D, C, G, B] = manipulator_eqs(q, qdot, params);

    % compute the control input
    % u1 = -100.0 * (q(1) - 0) - 10.0 * (qdot(1) - 0);
    % u2 = -100.0 * (q(2) - 0) - 10.0 * (qdot(2) - 0);  
    u1 = 0;
    u2 = 0;
    u = [u1; u2];

    % compute the dynamics
    qddot = D \ (-C * qdot + G + B * u);

    xdot = [qdot;
            qddot];
end

% get the manipulator equations, D(q) * qddot + C(q, qdot) * qdot + G(q) = B * u
% https://underactuated.csail.mit.edu/acrobot.html
function [D, C, G, B] = manipulator_eqs(q, qdot, params)

    % unpack the params
    m1 = params.m1;
    m2 = params.m2;
    l1 = params.l1;
    l2 = params.l2;
    g = params.g;

    % unpack the joint state
    q1 = q(1);
    q2 = q(2);
    q1dot = qdot(1);
    q2dot = qdot(2);

    % compute the inertia matrices
    l1_c = l1 / 2;  
    l2_c = l2 / 2;  
    I1 = m1 * l1_c^2; % point mass at center of the link
    I2 = m2 * l2_c^2; % point mass at center of the link
    % I1 = (1/3) * m1 * l1^2; % thin rod
    % I2 = (1/3) * m2 * l2^2; % thin rod

    % compute inertia matrix
    D11 = I1 + I2 + m2 * l1^2 + 2 * m2 * l1 *l2_c * cos(q2);
    D12 = I2 + m2 * l1 * l2_c * cos(q2);
    D21 = D12;
    D22 = I2;
    D = [D11, D12;
         D21, D22];

    % compute the coriolis matrix
    C11 = -2 * m2 * l1 * l2_c * sin(q2) * q2dot;
    C12 = -m2 * l1 * l2_c * sin(q2) * q2dot;
    C21 =  m2 * l1 * l2_c * sin(q2) * q1dot;
    C22 = 0;
    C_dyn = [C11, C12;
             C21, C22];
    C_damping = [params.b, 0;
                 0, params.b];
    C = C_dyn + C_damping;

    % compute the gravity vector
    G1 = -m1 * g * l1_c * sin(q1) - m2 * g * (l1 * sin(q1) + l2_c * sin(q1 + q2));
    G2 = -m2 * g * l2_c * sin(q1 + q2);
    G = [G1;
         G2];

    B = [1, 0;
         0, 1];
end