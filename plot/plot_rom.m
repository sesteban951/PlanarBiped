%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot ROM Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% import the data
file_location = '../data/';
t = importdata(file_location + "time.csv");

% unpack the output data
x = importdata(file_location + "rom_state.csv");
u = importdata(file_location + "rom_input.csv");

p = x(:,1);
v = x(:,2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% time window of interest
t_interval = [t(1), t(end)];
% t_interval = [1, 3];
idx = find(t >= t_interval(1) & t <= t_interval(2));

t = t(idx);

p = p(idx);
v = v(idx);
u = u(idx);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot the results
subplot(2, 3, 1);
grid on; hold on;
plot(t, p, 'LineWidth', 2);
xlabel('t (s)');
ylabel('p (m)');
title('p = p_c - p_s');

subplot(2, 3, 2);
grid on; hold on;
plot(t, v, 'LineWidth', 2);
xlabel('t (s)');
ylabel('v (m/s)');
title('v');

subplot(2, 3, 3);
grid on; hold on;
plot(t, u, 'LineWidth', 2);
xlabel('t (s)');
ylabel('u (m)');
title('u');

subplot(2, 3, [4, 6]);
grid on; hold on;
plot(p, v, 'k.', 'LineWidth', 2);
xlabel('p (m)');
ylabel('v (m/s)');
title('p vs. v');