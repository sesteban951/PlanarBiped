%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Manipulator Example
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% manipulator parameters
params.m1 = 1.0;  % mass of link 1
params.m2 = 1.0;  % mass of link 2
params.l1 = 1.0;  % length of link 1
params.l2 = 1.0;  % length of link 2
params.b1 = 0.1;   % damping
params.b2 = 0.1;   % damping
params.g = 9.81; % gravity

% initial conditions
x0 = [3.1;  % q1
      1.5;  % q2
      0.0;  % q1dot
      0.0]; % q2dot

% simulation
t_sim = 10.0;
dt = 0.025;
tspan = 0:dt:t_sim;

% simualte the dynamics
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[t, x] = ode45(@(t, x) dynamics(t, x, params), tspan, x0);

% extract the states
q = x(:, 1:2);
qdot = x(:, 3:4);

% precompute all the elbow and hand positions
p_elbow = zeros(length(t), 2);
p_hand = zeros(length(t), 2);
for i = 1:length(t)
    [p_elbow_, p_hand_] = forward_kin(q(i,:), params);
    p_elbow(i,:) = p_elbow_';
    p_hand(i,:) = p_hand_';
end

% plot the results
figure('Name', 'States');
set(gcf, 'WindowState', 'maximized');

subplot(2,2,1);
hold on; grid on;
plot(t, q(:,1), 'b', 'LineWidth', 1.5);
plot(t, q(:,2), 'r', 'LineWidth', 1.5);
yline(0);
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
legend('q1', 'q2');
title('Joint Angles');

subplot(2,2,3);
hold on; grid on;
plot(t, qdot(:,1), 'b', 'LineWidth', 1.5);
plot(t, qdot(:,2), 'r', 'LineWidth', 1.5);
yline(0);
xlabel('Time (s)');
ylabel('Joint Velocities (rad/s)');
legend('q1dot', 'q2dot');
title('Joint Velocities');

% animate the manipulator
px_max = max([p_hand(:,1)', p_elbow(:,1)', 0]);
px_min = min([p_hand(:,1)', p_elbow(:,1)', 0]);
pz_max = max([p_hand(:,2)', p_elbow(:,2)', 0]);
pz_min = min([p_hand(:,2)', p_elbow(:,2)', 0]);

subplot(2,2,[2, 4]);
hold on; grid on; axis equal;
xlim([px_min - 0.1, px_max + 0.1]);
ylim([pz_min - 0.1, pz_max + 0.1]);
xlabel('x (m)');
ylabel('z (m)');
xline(0); yline(0);

% plot the shoulder
plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

pause(1.0);

idx = 1;
tic;
while idx < length(t)
    
    % plot the first link
    link1 = plot([0, p_elbow(idx,1)], [0, p_elbow(idx,2)], 'k', 'LineWidth', 3);

    % plot the second link
    link2 = plot([p_elbow(idx,1), p_hand(idx,1)], [p_elbow(idx,2), p_hand(idx,2)], 'k', 'LineWidth', 3);

    % plot the elbow
    elbow = plot(p_elbow(idx,1), p_elbow(idx,2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    hand  = plot(p_hand(idx,1), p_hand(idx,2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    msg = sprintf('Time: %.3f', t(idx));
    title(msg);

    drawnow;

    % wait until the next time step
    while toc < t(idx+1)
        % do nothing
    end

    % increment the index
    if idx >= length(t)
        break;
    else
        idx = idx + 1;
        if idx == length(t)
            % dont delete
        else
            % delete the previous plot
            delete(link1);
            delete(link2);
            delete(elbow);
            delete(hand);
        end
    end
end

% manipulator dynamics
function xdot = dynamics(t, x, params)

    % unpack the state
    q    = [x(1); x(2)];  % joint angles
    qdot = [x(3); x(4)]; % joint velocities

    % get the manipulator equations
    [D, C, G, B] = manipulator_eqs(q, qdot, params);

    % compute the control input
    % u1 = -15.0 * (q(1) - 0) - 1.0 * (qdot(1) - 0);
    % u2 = -15.0 * (q(2) - 0) - 1.0 * (qdot(2) - 0);  
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
    % I1 = m1 * l1_c^2; % point mass at center of the link
    % I2 = m2 * l2_c^2; % point mass at center of the link
    I1 = (1/3) * m1 * l1^2; % thin rod
    I2 = (1/3) * m2 * l2^2; % thin rod

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
    C_damping = [params.b1, 0;
                 0,         params.b2];
    C = C_dyn + C_damping;

    % compute the gravity vector
    G1 = -m1 * g * l1_c * sin(q1) - m2 * g * (l1 * sin(q1) + l2_c * sin(q1 + q2));
    G2 = -m2 * g * l2_c * sin(q1 + q2);
    G = [G1;
         G2];

    B = [1, 0;
         0, 1];
end

% compute elbow and hand positions
function [p_elbow, p_hand] = forward_kin(q, params)

    % unpack the params
    l1 = params.l1;
    l2 = params.l2;

    % compute the elbow position
    p_elbow = [l1 * sin(q(1));
              -l1 * cos(q(1))];

    % compute the hand position
    p_hand = [l1 * sin(q(1)) + l2 * sin(q(1) + q(2));
             -l1 * cos(q(1)) - l2 * cos(q(1) + q(2))];
end