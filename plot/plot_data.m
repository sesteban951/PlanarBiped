%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Planar Bipde Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% import the data
file_location = '../data/';
t = importdata(file_location + "time.csv");
pos = importdata(file_location + "pos.csv");
vel = importdata(file_location + "vel.csv");

% extract the data
q_hip_left = pos(:,1);
q_knee_left = pos(:,2);
q_hip_right = pos(:,3);
q_knee_right = pos(:,4);
v_hip_left = vel(:,1);
v_knee_left = vel(:,2);
v_hip_right = vel(:,3);
v_knee_right = vel(:,4);

% prelim plotting
figure(1);

subplot(2,2,1);
hold on; yline(0);
plot(t, q_hip_left);
title('Left Hip Position');
xlabel('Time (s)');
grid on;

subplot(2,2,2);
hold on; yline(0);
plot(t, q_knee_left);
title('Left Knee Position');
xlabel('Time (s)');
grid on;

subplot(2,2,3);
hold on; yline(0);
plot(t, q_hip_right);
title('Right Hip Position');
xlabel('Time (s)');
grid on;

subplot(2,2,4);
hold on; yline(0);
plot(t, q_knee_right);
title('Right Knee Position');
xlabel('Time (s)');
grid on;