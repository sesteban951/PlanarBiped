%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Manipulator Example
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% manipulator parameters
params.m1 = 1.0;        % mass of link 1
params.m2 = 1.0;        % mass of link 2
params.l1 = 1.0;        % length of link 1
params.l2 = 1.0;        % length of link 2
params.b1 = 0.05;        % damping
params.b2 = 0.05;        % damping
params.g = 9.81;        % gravity

% manipulator gains
params.kp = 75.0;         % proportional gain for the manipulator
params.kd = 25.0;         % derivative gain for the manipulator

% single integrator parameters
params.k_single = 15.0; % gain of the single integrator

% desired trajectory (periodic ellipsoid trajectory)
params.rx = 0.25;        % radius x of the circle
params.rz = 0.5;         % radius z of the circle
params.c = [0.75; -1.0]; % center of the circle
params.T = 3.0;          % period of the circle

% flag to use the ROM framework or not
params.use_rom = 1; % 1 = use ROM (track single integrator (which tracks traj) by joint velocity control) 
                    % 0 = no ROM  (directly try to track the desired hand trajectory, using joint PD)

% initial condition of the Single Integrator (start at center of circle)
z0 = [params.c(1);  % px single integrator
      params.c(2)]; % pz single integrator

% initial condition of the manipulator
x0 = [1.5;  % q1
      -1.5;  % q2
      0.0;  % q1dot
      0.0]; % q2dot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIMULATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% simulation
t_sim = 10.0;        % simulation time
dt = 0.025;         % simualtion dt
tspan = 0:dt:t_sim; 

% simulate the dynamics
x_init = [z0; x0];
[t, x] = ode45(@(t, x) dynamics(t, x, params), tspan, x_init);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DATA EXTRACTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% extract the states
x_single = x(:, 1:2);
x_manip = x(:, 3:6);

% joint states of the manipulator
q = x_manip(:, 1:2);
qdot = x_manip(:, 3:4);

% precompute all the elbow and hand position and velocities
p_elbow = zeros(length(t), 2);
p_hand = zeros(length(t), 2);
pdot_hand = zeros(length(t), 2);
for i = 1:length(t)

    % joint states at time t
    q_t = q(i,:)';
    qdot_t = qdot(i,:)';

    % compute the forward kinematics
    [p_elbow_, p_hand_, pdot_hand_] = forward_kin(q_t, qdot_t, params);
    p_elbow(i,:) = p_elbow_';
    p_hand(i,:) = p_hand_';
    pdot_hand(i,:) = pdot_hand_';
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTTING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot the results
figure('Name', 'States');
% set(gcf, 'WindowState', 'maximized');

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
subplot(2,2,[2, 4]);
hold on; grid on; axis equal;

px_max = max([p_hand(:,1)', p_elbow(:,1)', x_single(:,1)', 0]);
px_min = min([p_hand(:,1)', p_elbow(:,1)', x_single(:,1)', 0]);
pz_max = max([p_hand(:,2)', p_elbow(:,2)', x_single(:,2)', 0]);
pz_min = min([p_hand(:,2)', p_elbow(:,2)', x_single(:,2)', 0]);

xlim([px_min - 0.2, px_max + 0.2]);
ylim([pz_min - 0.2, pz_max + 0.2]);
xlabel('x (m)');
ylabel('z (m)');
xline(0); yline(0);

% plot the shoulder
plot(0, 0, 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'r');

% plot the desired hand position
anlges = 0:0.01:2*pi;
px_hand = params.rx * cos(anlges) + params.c(1);
pz_hand = params.rz * sin(anlges) + params.c(2);
plot(px_hand, pz_hand, 'k--', 'LineWidth', 1);
plot(params.c(1), params.c(2), 'k+', 'MarkerSize', 20, 'LineWidth', 2.0);

pause(1.5);
idx = 1;
tic;
while idx < length(t)
    
    % plot the first and second links (black lines)
    link1 = plot([0, p_elbow(idx,1)], [0, p_elbow(idx,2)], 'k', 'LineWidth', 5);
    link2 = plot([p_elbow(idx,1), p_hand(idx,1)], [p_elbow(idx,2), p_hand(idx,2)], 'k', 'LineWidth', 5);

    % plot the elbow and hand (red circles)
    elbow = plot(p_elbow(idx,1), p_elbow(idx,2), 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
    hand  = plot(p_hand(idx,1), p_hand(idx,2), 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'r');

    % plot the single integrator position (blue asterisk)
    single_int = plot(x_single(idx,1), x_single(idx,2), 'b*', 'MarkerSize', 15, 'MarkerFaceColor', 'b', 'LineWidth', 2.0);

    % plot the desired hand position (green star)
    px_hand_des = params.rx * cos(2 * pi * t(idx) / params.T) + params.c(1);
    pz_hand_des = params.rz * sin(2 * pi * t(idx) / params.T) + params.c(2);
    hand_des = plot(px_hand_des, pz_hand_des, 'pentagram', 'MarkerSize', 15, 'MarkerFaceColor', 'g');

    % show the time in the title
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
            delete(hand_des);
            delete(single_int);
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% all system dynamics
function xdot = dynamics(t, x, params)

    % get the current desired hand position
    omega = 2 * pi / params.T;    % angular frequency
    p_hand_des = [params.rx * cos(omega * t) + params.c(1);
                  params.rz * sin(omega * t) + params.c(2)];
    pdot_hand_des = [-params.rx * omega * sin(omega * t);
                      params.rz * omega * cos(omega * t)];

    % Single Integrator dynamics
    z = [x(1); x(2)]; % position of the single integrator

    % compute the control input of the single integrator
    v = [-params.k_single * (z(1) - p_hand_des(1));
         -params.k_single * (z(2) - p_hand_des(2))];

    % unpack the manipulator state
    x_manip = x(3:6); % joint states of the manipulator
    q    = [x_manip(1); x_manip(2)];  % joint angles
    qdot = [x_manip(3); x_manip(4)];  % joint velocities

    % use reduced order model input
    if params.use_rom == 1

        % compute the forward kineamtics to get the ROM states
        [~, p_hand, ~] = forward_kin(q, qdot, params);
        z_ = p_hand;
        v_ = [-params.k_single * (z_(1) - p_hand_des(1));
              -params.k_single * (z_(2) - p_hand_des(2))];

        % compute IK using the single integrator position
        % [~, qdot_des] = inverse_kin(z, v, params);  % (this one is wrong?)
        [~, qdot_des] = inverse_kin(z_, v_, params);

        % compute joint torques
        u1 = - params.kd * (qdot(1) - qdot_des(1));
        u2 = - params.kd * (qdot(2) - qdot_des(2));
        u = [u1; u2];
        
    % use naive control input (no ROM)
    else
        % compute IK using the desired hand position directly
        [q_des, qdot_des] = inverse_kin(p_hand_des, pdot_hand_des, params);

        % compute joint torques
        u1 = -params.kp * (q(1) - q_des(1)) - params.kd * (qdot(1) - qdot_des(1));
        u2 = -params.kp * (q(2) - q_des(2)) - params.kd * (qdot(2) - qdot_des(2));
        u = [u1; u2];
    end

    % compute the manipulator dynamics
    [D, C, G, B] = manipulator_eqs(q, qdot, params);
    qddot = D \ (-C * qdot - G + B * u);

    % return the whole system dynamics
    xdot = [v;
            qdot;
            qddot];
end

% forward kinematics of the manipulator
function [p_elbow, p_hand, pdot_hand] = forward_kin(q, qdot, params)

    % unpack the params
    l1 = params.l1;
    l2 = params.l2;

    % compute the elbow position
    p_elbow = [l1 * sin(q(1));
              -l1 * cos(q(1))];

    % compute the hand position
    p_hand = [l1 * sin(q(1)) + l2 * sin(q(1) + q(2));
             -l1 * cos(q(1)) - l2 * cos(q(1) + q(2))];

    % compute the hand velocity
    J = manipulator_jacobian(q, params);
    pdot_hand = J * qdot;
end

% inverse kinematics of the manipulator
function [q_sol, qdot_sol] = inverse_kin(p_hand_des, pdot_hand_des, params)

    % unpack the params
    l1 = params.l1;
    l2 = params.l2;

    % compute the inverse kinematics
    x = p_hand_des(1);
    z = p_hand_des(2);
    L = sqrt(x^2 + z^2);

    % elbow angle (be aware that there are two solutions, elbow up and elbow down)
    gamma = acos((L^2 - l1^2 - l2^2) / (-2 * l1 * l2));
    q2 = gamma - pi;
    
    % shoulder angle
    beta = atan2(x, -z);
    alpha = acos((l2^2 - l1^2 - L^2) / (-2 * l1 * L));
    q1 = beta + alpha;
    
    % IK solution 
    q_sol = [q1; q2];

    % compute the joint velocities via inverse Jacobian
    J = manipulator_jacobian(q_sol, params);
    qdot_sol = J \ pdot_hand_des;
end

% manipulator jacobian
function J = manipulator_jacobian(q, params)

    % extract the params
    l1 = params.l1;
    l2 = params.l2;

    % extract the joint angles
    q1 = q(1);
    q2 = q(2);

    % compute the jacobian
    J = [l1 * cos(q1) + l2 * cos(q1 + q2), l2 * cos(q1 + q2);
         l1 * sin(q1) + l2 * sin(q1 + q2), l2 * sin(q1 + q2)];
end

% compute the robot equations, D(q) * qddot + C(q, qdot) * qdot + G(q) = B * u
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

    % compute the inertias of the links
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
    G = -[G1;
          G2];

    % compute the input matrix
    B = [1, 0;
         0, 1];
end
