%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Joint Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% import the data
file_location = '../data/';
t = importdata(file_location + "time.csv");
q_des = importdata(file_location + "q_des.csv");
q_act = importdata(file_location + "q_act.csv");

q_HL = q_act(:,1);
q_KL = q_act(:,2);
q_HR = q_act(:,3);
q_KR = q_act(:,4);
q_HL_des = q_des(:,1);
q_KL_des = q_des(:,2);
q_HR_des = q_des(:,3);
q_KR_des = q_des(:,4);

% plot the data
figure(1);
subplot(2,2,1);
grid on; hold on; 
plot(t, q_HL_des, 'b', 'LineWidth', 2);
plot(t, q_HL, 'r', 'LineWidth', 2);
xlabel('Time [s]');
title('Left Hip');
legend('Desired', 'Actual');

subplot(2,2,2);
grid on; hold on;
plot(t, q_KL_des, 'b', 'LineWidth', 2);
plot(t, q_KL, 'r', 'LineWidth', 2);
xlabel('Time [s]');
title('Left Knee');
legend('Desired', 'Actual');

subplot(2,2,3);
grid on; hold on;
plot(t, q_HR_des, 'b', 'LineWidth', 2);
plot(t, q_HR, 'r', 'LineWidth', 2);
xlabel('Time [s]');
title('Right Hip');
legend('Desired', 'Actual');

subplot(2,2,4);
grid on; hold on;
plot(t, q_KR_des, 'b', 'LineWidth', 2);
plot(t, q_KR, 'r', 'LineWidth', 2);
xlabel('Time [s]');
title('Right Knee');
legend('Desired', 'Actual');

