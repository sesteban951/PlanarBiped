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

% random q
q1_lims = [-pi/2, pi/2];
q2_lims = [-pi/2, 0];
q1 = unifrnd(q1_lims(1), q1_lims(2));
q2 = unifrnd(q2_lims(1), q2_lims(2));
% q = [q1; q2];
q = [0.5; -0.7];
qdot = [0; 0];

% syntehsize a joint trajectory for the robot
t = 0: 0.1 : 3.0;
q_t = zeros(2, length(t));
for i = 1:length(t)

    % sine wave
    q1 = 0.5 * sin(2 * pi * t(i)) + 0.0;
    q2 = 0.25 * sin(2 * pi * t(i)) - 0.5;
    q_t = [q1; q2];

    qdot1 = 2 * pi * 0.5 * cos(2 * pi * t(i));
    qdot2 = 2 * pi * 0.25 * cos(2 * pi * t(i));
    qdot_t = [qdot1; qdot2];
    

    % populate
    q_t(:, i) = q_t;
end

% compute the forward kinematics



% plot the robot
figure;
axis equal; hold on; grid on;
xline(0);
yline(0);
px_max = max([p_foot(1), p_knee(1), 0]);
px_min = min([p_foot(1), p_knee(1), 0]);
pz_max = max([p_foot(2), p_knee(2), 0]);
pz_min = min([p_foot(2), p_knee(2), 0]);
xlim([px_min - 0.1, px_max + 0.1]);
ylim([pz_min - 0.1, pz_max + 0.1]);
xlabel('x (m)');
ylabel('z (m)');

% plot the thigh and shin
plot([0, p_knee(1)], [0, p_knee(2)], 'k', 'LineWidth', 2);
plot([p_knee(1), p_foot(1)], [p_knee(2), p_foot(2)], 'k', 'LineWidth', 2);

% plot the hip, knee and foot
plot(0, 0, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
plot(p_knee(1), p_knee(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
plot(p_foot(1), p_foot(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');




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

% % compute the inverse kinematics
% function [q, qdot] = inv_kinematics(p_foot, v_foot, params)

%     % link lengths
%     l1 = params.l1;
%     l2 = params.l2;

%     % unpack stuff
%     px = p_foot(1);
%     pz = p_foot(2);

%     % Inverse kinematics
%     q1 = atan2(px, -pz);
%     q2 = acos((px^2 + pz^2 - l1^2 - l2^2) / (2 * l1 * l2));

%     % Jacobian
%     J = [l1 * cos(q1) + l2 * cos(q1 + q2), l2 * cos(q1 + q2);
%          l1 * sin(q1) + l2 * sin(q1 + q2), l2 * sin(q1 + q2)];
%     qdot = pinv(J) * v_foot;
% end

function q = inv_kinematics(p_foot_des, params)
    % Extract link lengths
    l1 = params.l1;
    l2 = params.l2;
    
    % Extract desired foot position
    x = p_foot_des(1);
    y = p_foot_des(2);

    % Compute q2 using the law of cosines
    c2 = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2);
    s2 = sqrt(1 - c2^2);  % Choose positive for elbow-up, negative for elbow-down
    q2 = atan2(s2, c2);

    % Compute q1 using the law of sines
    gamma = atan2(y, x);
    beta = atan2(l2 * sin(q2), l1 + l2 * cos(q2));
    q1 = gamma - beta;

    % Return joint angles
    q = [q1; q2];
end