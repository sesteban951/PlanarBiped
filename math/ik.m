%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Planar Bipde Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% link lengths
l0 = 0.40; % torso length
l1 = 0.25; % thigh length
l2 = 0.25; % shin length
params.l0 = l0;
params.l1 = l1;
params.l2 = l2;

animation = 1;

% syntehsize a joint trajectory for the robot
t = 0: 0.025 : 7.0;
q_t = zeros(2, length(t));
qdot_t = zeros(2, length(t));

f = 0.5;
w = 2 * pi * f;

for i = 1:length(t)

    % sine wave
    q1 = 1.0 * sin(w * t(i)) + 0.5;
    q2 = 1.0 * sin(w * t(i)) - 1.0;
    qdot1 = 2 * pi * 0.5 * cos(w * t(i));
    qdot2 = 2 * pi * 0.25 * cos(w * t(i));
    
    % populate
    q_t(:, i) = [q1; q2];
    qdot_t(:, i) = [qdot1; qdot2];
end


% compute the forward kinematics
p_knee_t = zeros(2, length(t));
p_foot_t = zeros(2, length(t));
v_foot_t = zeros(2, length(t));
for i = 1:length(t)
    [p_knee_t(:, i), p_foot_t(:, i), v_foot_t(:, i)] = fwd_kinmeatics(q_t(:, i), qdot_t(:,i), params);
end

% compute the inverse kinematics
q_t_inv = zeros(2, length(t));
for i = 1:length(t)
    q_t_inv(:, i) = inv_kinematics(p_foot_t(:, i), params);
end

% plot the joint angles
if animation == 0
    figure('Name', 'Joint Angles');
    subplot(2, 1, 1);
    hold on; grid on;
    plot(t, q_t(1, :), 'r', 'LineWidth', 2);
    plot(t, q_t_inv(1, :), 'b', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Joint Angle (rad)');
    title('Hip');
    legend('Forward', 'Inverse');

    subplot(2, 1, 2);
    hold on; grid on;
    plot(t, q_t(2, :), 'r', 'LineWidth', 2);
    plot(t, q_t_inv(2, :), 'b', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Joint Angle (rad)');
    title('Knee');
    legend('Forward', 'Inverse');
end

% plot the robot
if animation == 1
    figure('Name', 'Robot');
    axis equal; hold on; grid on;
    xline(0);
    yline(0);
    px_max = max([p_foot_t(1,:), p_knee_t(1,:), 0]);
    px_min = min([p_foot_t(1,:), p_knee_t(1,:), 0]);
    pz_max = max([p_foot_t(2,:), p_knee_t(2,:), 0]);
    pz_min = min([p_foot_t(2,:), p_knee_t(2,:), 0]);
    xlim([px_min - 0.1, px_max + 0.1]);
    ylim([pz_min - 0.1, pz_max + 0.1]);
    xlabel('x (m)');
    ylabel('z (m)');

    % plot the hip
    plot(0, 0, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');

    idx = 1;
    tic;
    while idx < length(t)

        % plot the thigh and shin
        thigh = plot([0, p_knee_t(1, idx)], [0, p_knee_t(2, idx)], 'k', 'LineWidth', 2);
        shin = plot([p_knee_t(1, idx), p_foot_t(1, idx)], [p_knee_t(2, idx), p_foot_t(2, idx)], 'k', 'LineWidth', 2);

        % plot the knee and foot
        knee = plot(p_knee_t(1, idx), p_knee_t(2, idx), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
        foot = plot(p_foot_t(1, idx), p_foot_t(2, idx), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');

        msg = sprintf('Time: %.3f', t(idx));
        title(msg);

        drawnow;

        % wait until next iteration
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
                delete(thigh);
                delete(shin);
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
function [p_knee, p_foot, v_foot] = fwd_kinmeatics(q, qdot, params)

    % link lengths
    l1 = params.l1;
    l2 = params.l2;

    % unpack stuff
    q1 = q(1);
    q2 = q(2);

    % Forward kinematics
    p_knee = [l1 * sin(q1);
             -l1 * cos(q1)];
    p_foot = [l1 * sin(q1) + l2 * sin(q1 + q2);
             -l1 * cos(q1) - l2 * cos(q1 + q2)];

    % Jacobian
    J = [l1 * cos(q1) + l2 * cos(q1 + q2), l2 * cos(q1 + q2);
         l1 * sin(q1) + l2 * sin(q1 + q2), l2 * sin(q1 + q2)];
    v_foot = J * qdot;
end


function q = inv_kinematics(p_foot_des, params)
    % Extract link lengths
    l1 = params.l1;
    l2 = params.l2;
    
    % Extract desired foot position
    x = p_foot_des(1);
    z = p_foot_des(2);

    % Compute q2 using the law of cosines
    c2 = (x^2 + z^2 - l1^2 - l2^2) / (2 * l1 * l2);
    s2 = -sqrt(1 - c2^2);  % Choose positive for knee-up, negative for knee-down
    q2 = atan2(s2, c2);

    % Compute q1 using the law of sines
    gamma = atan2(z, x);
    beta = atan2(l2 * sin(q2), l1 + l2 * cos(q2));
    q1 = gamma - beta + pi/2;

    % Return joint angles
    q = [q1; q2];
end
