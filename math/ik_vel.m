%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Planar Bipde Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% link lengths
l1 = 0.5; % thigh length
l2 = 0.5; % shin length
params.l1 = l1;
params.l2 = l2;
g = 2.0; % gravity

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% compute the ballistic tajectory
dt = 0.02;
tmax = 3.0;
tspan = 0: dt : tmax;

% assume floating base has some ballistic dynamics
px_0 = 0.0; % initial x position
pz_0 = 1.0; % initial z position
theta0 = pi/2; % initial angle
vx_0 = 1.0; % initial x velocity
vz_0 = 3.0; % initial z velocity
thetadot0 = -2.0; % initial angular velocity

% compute some open loop leg trajectories
% hips
hip_max = 0.5;
hip_min = -0.5;
A_hip = (hip_max - hip_min) / 2;
hip_offset = (hip_max + hip_min) / 2;

% knees
knee_max = -0.1;
knee_min = -1.57;
A_knee = (knee_max - knee_min) / 2;
knee_offset = (knee_max + knee_min) / 2;

% frequency 
f = 0.5;
w = 2 * pi * f;

q_t = zeros(2, length(tspan));
qdot_t = zeros(2, length(tspan));
for i = 1:length(tspan)

    % current time
    t = tspan(i);

    % sine wave
    q1 = A_hip * sin(w * t) + hip_offset;
    q2 = A_knee * sin(w * t) + knee_offset;
    % q2 = 0;
    qdot1 = A_hip * w * cos(w * t);
    qdot2 = A_knee * w * cos(w * t);
    % qdot2 = 0;

    % populate
    q_t(:, i) = [q1 ; q2];
    qdot_t(:, i) = [qdot1; qdot2];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_base = zeros(length(tspan), 6);
x_leg = zeros(length(tspan), 6);
for i = 1:length(tspan)
    
    % compute the state
    t = tspan(i);
    px_t = px_0 + vx_0 * t;
    pz_t = pz_0 + vz_0 * t - 0.5 * g * t^2;
    vx_t = vx_0;
    vz_t = vz_0 - g * t;

    theta = theta0 + thetadot0 * t;
    thetadot = thetadot0;

    % store the base state
    x_base(i, :) = [px_t, pz_t, theta, vx_t, vz_t, thetadot];

    % compute the foot position
    p_base = [px_t, pz_t, theta];
    v_base = [vx_t, vz_t, thetadot];
    q = q_t(:, i);
    qdot = qdot_t(:, i);
    [p_knee_W, p_foot_W, v_foot_W] = fwd_kinematics(p_base, v_base, q, qdot, params);

    % store the foot state
    x_leg_ = [p_knee_W(1), p_knee_W(2), p_foot_W(1), p_foot_W(2), v_foot_W(1), v_foot_W(2)];
    x_leg(i, :) = x_leg_;
end

% for computing IK stuff
p_base_W = x_base(:, 1:3);
p_foot_W = x_leg(:, 3:4);
v_base_W = x_base(:, 4:6);
v_foot_W = x_leg(:, 5:6);

q_ik = zeros(2, length(tspan));
qdot_ik = zeros(2, length(tspan));
for i = 1:length(tspan)

    % compute the desired base position
    p_base_des_W_ = p_base_W(i, :)';
    p_foot_des_W_ = p_foot_W(i, :)';
    v_base_des_W_ = v_base_W(i, :)';
    v_foot_des_W_ = v_foot_W(i, :)';

    % compute the inverse kinematics
    [q_sol, qdot_sol] = inv_kinematics(p_base_des_W_, p_foot_des_W_, v_base_des_W_, v_foot_des_W_, params);

    % store the solution
    q_ik(:, i) = q_sol;
    qdot_ik(:, i) = qdot_sol;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

animation = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot data
if animation == 0

    % to verify the forward velocity kinematics
    px_foot_W = x_leg(:, 3);
    pz_foot_W = x_leg(:, 4);
    vx_foot_W = x_leg(:, 5);
    vz_foot_W = x_leg(:, 6);
    vx_foot_diff = diff(px_foot_W) / dt;
    vz_foot_diff = diff(pz_foot_W) / dt;

    figure;
    subplot(1,2,1);
    hold on; grid on;
    plot(tspan, vx_foot_W, 'r', 'LineWidth', 2);
    plot(tspan(1:end-1), vx_foot_diff, 'b', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Z Foot Velocity (m/s)');

    subplot(1,2,2);
    hold on; grid on;
    plot(tspan, vz_foot_W, 'r', 'LineWidth', 2);
    plot(tspan(1:end-1), vz_foot_diff, 'b', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Y Foot Velocity (m/s)');

    figure;
    subplot(2,2,1);
    hold on; grid on;
    plot(tspan, q_t(1, :), 'r', 'LineWidth', 2);
    plot(tspan, q_ik(1, :), 'b', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Hip Angle (rad)');
    
    subplot(2,2,2);
    hold on; grid on;
    plot(tspan, q_t(2, :), 'r', 'LineWidth', 2);
    plot(tspan, q_ik(2, :), 'b', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Knee Angle (rad)');

    subplot(2,2,3);
    hold on; grid on;
    plot(tspan, qdot_t(1, :), 'r', 'LineWidth', 2);
    plot(tspan, qdot_ik(1, :), 'b', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Hip Velocity (rad/s)');
    
    subplot(2,2,4);
    hold on; grid on;
    plot(tspan, qdot_t(2, :), 'r', 'LineWidth', 2);
    plot(tspan, qdot_ik(2, :), 'b', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Knee Velocity (rad/s)');
end

% plot the ballistic trajectory
if animation == 1
    figure;
    hold on; axis equal; grid on;
    xline(0); yline(0);
    plot(x_base(:, 1), x_base(:, 2), 'k--', 'LineWidth', 1);

    idx = 1;
    tic;
    while idx < length(tspan)

        % positions
        px_base = x_base(idx, 1);
        pz_base = x_base(idx, 2);
        theta = x_base(idx, 3);
        px_knee = x_leg(idx, 1);
        pz_knee = x_leg(idx, 2);
        px_foot = x_leg(idx, 3);
        pz_foot = x_leg(idx, 4);

        % plot the axis
        [x_axis_, y_axis_] = axis_rigid_transform(theta);
        x_axis_plot = plot([px_base, px_base + x_axis_(1)], [pz_base, pz_base + x_axis_(2)], 'r', 'LineWidth', 2);
        y_axis_plot = plot([px_base, px_base + y_axis_(1)], [pz_base, pz_base + y_axis_(2)], 'b', 'LineWidth', 2);

        % plot the thigh and shin
        thigh = plot([px_base, px_knee], [pz_base, pz_knee], 'k', 'LineWidth', 2);
        shin = plot([px_knee, px_foot], [pz_knee, pz_foot], 'k', 'LineWidth', 2);
        hip = plot(px_base, pz_base, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        knee = plot(px_knee, pz_knee, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        foot = plot(px_foot, pz_foot, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

        msg = sprintf('Time: %.3f', tspan(idx));
        title(msg);

        drawnow;

        % wait until next iteration
        while toc < tspan(idx+1)
            % do nothing
        end
        
        % increment the index
        if idx >= length(tspan)
            break;
        else
            idx = idx + 1;
            if idx == length(tspan)
                % dont delete
            else
                delete(x_axis_plot);
                delete(y_axis_plot);
                delete(thigh);
                delete(shin);
                delete(hip);
                delete(knee);
                delete(foot);
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AUXILIARY FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% compute the forward kinematics
function [p_knee_W, p_foot_W, v_foot_W] = fwd_kinematics(p_base, v_base, q, qdot, params)

    % link lengths
    l1 = params.l1;
    l2 = params.l2;

    % unpack stuff
    p_B = p_base(1:2)';
    v_B = v_base(1:2)';
    theta = p_base(3);
    thetadot = v_base(3);
    q1 = q(1);
    q2 = q(2);
    
    % compute the rotation matrix
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];

    % Forward kinematics
    p_knee_B = [l1 * sin(q1);
               -l1 * cos(q1)];
    p_foot_B = [l1 * sin(q1) + l2 * sin(q1 + q2);
               -l1 * cos(q1) - l2 * cos(q1 + q2)];
    p_knee_W = p_B + R * p_knee_B;
    p_foot_W = p_B + R * p_foot_B;

    % Jacobian
    J = [l1 * cos(q1) + l2 * cos(q1 + q2), l2 * cos(q1 + q2);
         l1 * sin(q1) + l2 * sin(q1 + q2), l2 * sin(q1 + q2)];
    v_foot_B = J * qdot;

    % Angular velocity cross-product term
    omega_skew = thetadot * [0 -1; 1 0];  % Skew-symmetric matrix in 2D
    Rdot = omega_skew * R;

    % Compute foot velocity in world frame
    v_foot_W = v_B + Rdot* p_foot_B + R * v_foot_B; % <----------------- Implement this
end

% compute the inverse kinematics
function [q_sol, qdot_sol] = inv_kinematics(p_base_des_W, p_foot_des_W, pdot_base_des_W, pdot_foot_des_W, params)
    
    % Extract link lengths
    l1 = params.l1;
    l2 = params.l2;
    
    % unacpk desired base position in world frame
    theta = p_base_des_W(3);
    thetadot = pdot_base_des_W(3);
    p_base_des_W = p_base_des_W(1:2);
    v_base_des_W = pdot_base_des_W(1:2);

    % desired foot position in base frame
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];
    p_foot_des_B = R' * (p_foot_des_W - p_base_des_W);

    omega_skew = thetadot * [0 -1; 1 0];
    Rdot = omega_skew * R;
    v_foot_des_B = R' * (-v_base_des_W - Rdot * p_foot_des_B + pdot_foot_des_W); % <----------------- Implement this

    % Extract desired foot position
    x = p_foot_des_B(1);
    z = p_foot_des_B(2);
    xdot = v_foot_des_B(1);
    zdot = v_foot_des_B(2);

    % https://www.youtube.com/watch?v=jyQyusoxlW8
    % https://www.youtube.com/watch?v=RH3iAmMsolo
    % finding the knee joint
    L = sqrt(x^2 + z^2);

    % knee angle (be aware that there are two solutions)
    gamma = acos((L^2 - l1^2 - l2^2) / (-2 * l1 * l2));
    q2 = pi - gamma;
    q2 = -q2;
    
    % hip angle
    beta = atan2(x, -z);
    alpha = acos((l2^2 - l1^2 - L^2) / (-2 * l1 * L));
    q1 = beta + alpha;
    
    % solution 
    q_sol = [q1; q2];

    % compute joint velocities by Jacobian
    J = [l1 * cos(q1) + l2 * cos(q1 + q2), l2 * cos(q1 + q2);
         l1 * sin(q1) + l2 * sin(q1 + q2), l2 * sin(q1 + q2)];
    qdot_sol = J \ [xdot; zdot];

    % qdot_sol = [qdot1; qdot2];
end

% perforam a rigid transform on the axis
function [x_axis, y_axis] = axis_rigid_transform(theta)
    
    % rigid frame quivers
    axis_mag = 0.5;
    x_axis_ = [1; 0] * axis_mag;
    y_axis_ = [0; 1] * axis_mag;

    % axis rotation
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];
    x_axis = R * x_axis_;
    y_axis = R * y_axis_;

end