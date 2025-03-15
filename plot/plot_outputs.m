%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Joint Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% import the data
file_location = '../data/';
t = importdata(file_location + "time.csv");

% unpack the output data
y_des = importdata(file_location + "y_des.csv");
y_act = importdata(file_location + "y_act.csv");
q_des = importdata(file_location + "q_des.csv");
q_act = importdata(file_location + "q_act.csv");

% unpack the output data
y_base_des = y_des(:, 1:3);
y_left_des = y_des(:, 4:5);
y_right_des = y_des(:, 6:7);

y_base_act = y_act(:, 1:3);
y_left_act = y_act(:, 4:5);
y_right_act = y_act(:, 6:7);

% unpack the joint data
q_HL = q_act(:,1);
q_KL = q_act(:,2);
q_HR = q_act(:,3);
q_KR = q_act(:,4);
q_HL_des = q_des(:,1);
q_KL_des = q_des(:,2);
q_HR_des = q_des(:,3);
q_KR_des = q_des(:,4);

% unpack the joint data
t_phase = importdata(file_location + "t_phase.csv");
stance = importdata(file_location + "stance.csv");
p_stance = importdata(file_location + "stance_foot.csv");
p_swing_init = importdata(file_location + "swing_init.csv");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% unload some stuff from the yaml config file
config_file_path = "../config/biped.yaml";
config = yaml.loadFile(config_file_path);

% some parameters
T_SSP = config.HLIP.T_SSP;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% time window of interest
t_interval = [t(1), t(end)];
% t_interval = [1, 3];
idx = find(t >= t_interval(1) & t <= t_interval(2));

t = t(idx);

y_base_des = y_base_des(idx, :);
y_left_des = y_left_des(idx, :);
y_right_des = y_right_des(idx, :);

y_base_act = y_base_act(idx, :);
y_left_act = y_left_act(idx, :);
y_right_act = y_right_act(idx, :);

q_HL_des = q_HL_des(idx);
q_KL_des = q_KL_des(idx);
q_HR_des = q_HR_des(idx);
q_KR_des = q_KR_des(idx);

q_HL = q_HL(idx);
q_KL = q_KL(idx);
q_HR = q_HR(idx);
q_KR = q_KR(idx);

t_phase = t_phase(idx);
stance = stance(idx);

n_steps = floor(t(end) / T_SSP);
t_SSP = 0: T_SSP : n_steps*T_SSP;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% figure('Name', 'Output Data');

% tabgroup = uitabgroup('Position', [0 0 1 1]);

% tab_outputs = uitab(tabgroup, 'Title', 'Outputs');
% ax_outputs = axes('Parent', tab_outputs);

% subplot(3, 3, 1);
% grid on; hold on;
% plot(t, y_base_des(:, 1), 'b', 'LineWidth', 2);
% plot(t, y_base_act(:, 1), 'r', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('X Position [m]');
% title('Base px');
% legend('Desired', 'Actual', 'Location', 'best');

% subplot(3, 3, 4);
% grid on; hold on;
% plot(t, y_base_des(:, 2), 'b', 'LineWidth', 2);
% plot(t, y_base_act(:, 2), 'r', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Z Position [m]');
% title('Base pz');
% legend('Desired', 'Actual', 'Location', 'best');

% subplot(3, 3, 7);
% grid on; hold on;
% plot(t, y_base_des(:, 3), 'b', 'LineWidth', 2);
% plot(t, y_base_act(:, 3), 'r', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Theta [rad]');
% title('Base theta');
% legend('Desired', 'Actual', 'Location', 'best');

% subplot(3, 3, 2);
% grid on; hold on;
% plot(t, y_left_des(:, 1), 'b.', 'LineWidth', 2);
% plot(t, y_left_act(:, 1), 'r.', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('X Position [m]');
% title('Left px');
% legend('Desired', 'Actual', 'Location', 'best');

% subplot(3, 3, 5);
% grid on; hold on;
% plot(t, y_left_des(:, 2), 'b', 'LineWidth', 2);
% plot(t, y_left_act(:, 2), 'r', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Z Position [m]');
% title('Left pz');
% legend('Desired', 'Actual', 'Location', 'best');

% subplot(3, 3, 3);
% grid on; hold on;
% plot(t, y_right_des(:, 1), 'b.', 'LineWidth', 2);
% plot(t, y_right_act(:, 1), 'r.', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('X Position [m]');
% title('Right px');
% legend('Desired', 'Actual', 'Location', 'best');

% subplot(3, 3, 6);
% grid on; hold on;
% plot(t, y_right_des(:, 2), 'b', 'LineWidth', 2);
% plot(t, y_right_act(:, 2), 'r', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Z Position [m]');
% title('Right pz');
% legend('Desired', 'Actual', 'Location', 'best');

% subplot(3, 3, 8);
% grid on; hold on;
% plot(y_left_des(:, 1), y_left_des(:, 2), 'b.', 'LineWidth', 2);
% plot(y_left_act(:, 1), y_left_act(:, 2), 'r.', 'LineWidth', 2);
% xlabel('X Position [m]');
% ylabel('Z Position [m]');
% title('Left Leg');
% legend('Desired', 'Actual', 'Location', 'best');

% subplot(3, 3, 9);
% grid on; hold on;
% plot(y_right_des(:, 1), y_right_des(:, 2), 'b.', 'LineWidth', 2);
% plot(y_right_act(:, 1), y_right_act(:, 2), 'r.', 'LineWidth', 2);
% xlabel('X Position [m]');
% ylabel('Z Position [m]');
% title('Right Leg');
% legend('Desired', 'Actual', 'Location', 'best');

% tab_joints = uitab(tabgroup, 'Title', 'Joints');
% ax_joints = axes('Parent', tab_joints);

% subplot(2,2,1);
% grid on; hold on; 
% plot(t, q_HL_des, 'b', 'LineWidth', 2);
% plot(t, q_HL, 'r', 'LineWidth', 2);
% xlabel('Time [s]');
% title('Left Hip');
% legend('Desired', 'Actual');

% subplot(2,2,2);
% grid on; hold on;
% plot(t, q_KL_des, 'b', 'LineWidth', 2);
% plot(t, q_KL, 'r', 'LineWidth', 2);
% xlabel('Time [s]');
% title('Left Knee');
% legend('Desired', 'Actual');

% subplot(2,2,3);
% grid on; hold on;
% plot(t, q_HR_des, 'b', 'LineWidth', 2);
% plot(t, q_HR, 'r', 'LineWidth', 2);
% xlabel('Time [s]');
% title('Right Hip');
% legend('Desired', 'Actual');

% subplot(2,2,4);
% grid on; hold on;
% plot(t, q_KR_des, 'b', 'LineWidth', 2);
% plot(t, q_KR, 'r', 'LineWidth', 2);
% xlabel('Time [s]');
% title('Right Knee');
% legend('Desired', 'Actual');

% tab_phase = uitab(tabgroup, 'Title', 'Phase');
% ax_phase = axes('Parent', tab_phase);

% grid on; hold on;
% plot(t, t_phase, 'b', 'LineWidth', 2);
% for i = 1:length(t_SSP)
%     xline(t_SSP(i), '--r', 'LineWidth', 2);
% end
% plot(t, stance*0.4, 'g', 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Phase [s]');
% legend('Phase', 'Switch', 'Location', 'best');
% title('Phase');

figure('Name', 'Foot Comparison');
hold on; 
plot(t, y_left_des(:, 1), 'bo', "MarkerSize", 5, 'LineWidth', 2);
plot(t, y_right_des(:, 1), 'rx', "MarkerSize", 1, 'LineWidth', 2);
for i = 1:length(t_SSP)
    xline(t_SSP(i), 'k--', 'LineWidth', 1);
end

figure;
hold on;
plot(t, p_stance(:,1),'bo', "MarkerSize", 2, 'LineWidth', 2);
plot(t, p_swing_init(:,1),'ro', "MarkerSize", 2, 'LineWidth', 2);
for i = 1:length(t_SSP)
    xline(t_SSP(i), 'k--', 'LineWidth', 1);
end