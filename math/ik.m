%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Planar Bipde Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% link lengths
l0 = 0.40; % torso length
l1 = 0.5; % thigh length
l2 = 0.5; % shin length
params.l0 = l0;
params.l1 = l1;
params.l2 = l2;

% animation = 1 means animate the robot
% animation = 0 means plot the joint angles
animation = 1;

% syntehsize a joint trajectory for the robot
t = 0: 0.025 : 7.0;
q_t = zeros(2, length(t));
qdot_t = zeros(2, length(t));

f = 0.5;
w = 2 * pi * f;

% hips
hip_max = 1.58;
hip_min = -1.58;
A_hip = (hip_max - hip_min) / 2;
hip_offset = (hip_max + hip_min) / 2;

% knees
knee_max = 0.0;
knee_min = -1.56;
A_knee = (knee_max - knee_min) / 2;
knee_offset = (knee_max + knee_min) / 2;

for i = 1:length(t)

    % sine wave
    q1 = A_hip * sin(w * t(i)) + hip_offset;
    q2 = A_knee * sin(w * t(i)) + knee_offset;
    qdot1 = A_hip * w * cos(w * t(i));
    qdot2 = A_knee * w * cos(w * t(i));

    % populate
    q_t(:, i) = [q1 ; q2];
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
    plot(t, q_t_inv(1, :), 'b', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Joint Angle (rad)');
    title('Hip');
    legend('Forward', 'Inverse');

    subplot(2, 1, 2);
    hold on; grid on;
    plot(t, q_t(2, :), 'r', 'LineWidth', 2);
    plot(t, q_t_inv(2, :), 'b', 'LineWidth', 1);
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

function q_sol = inv_kinematics(p_foot_des, params)
    % Extract link lengths
    l1 = params.l1;
    l2 = params.l2;
    
    % Extract desired foot position
    x = p_foot_des(1);
    z = p_foot_des(2);

    % Chat GPT
    % Compute q2 using the law of cosines
    % c2 = (x^2 + z^2 - l1^2 - l2^2) / (2 * l1 * l2);
    % s2 = -sqrt(1 - c2^2);  % Choose positive for knee-up, negative for knee-down
    % q2 = atan2(s2, c2);

    % % Compute q1 using the law of sines
    % gamma = atan2(z, x);
    % beta = atan2(l2 * sin(q2), l1 + l2 * cos(q2));
    % q1 = gamma - beta + pi/2;

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
end
