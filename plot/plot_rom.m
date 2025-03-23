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

% extract some data from the yaml config file
config_file_path = "../config/biped.yaml";
config = yaml.loadFile(config_file_path);

% some parameters
g = 9.81;
z0_des = config.HLIP.z0;
v_des = config.HLIP.v_des;

T_SSP = config.HLIP.T_SSP;
T_DSP = 0.0;
T_tot = T_SSP + T_DSP;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROBOT Emperical

% time window of interest
% t_interval = [t(1), t(end)];
t_interval = [t(end) - 2, t(end)];
idx = find(t >= t_interval(1) & t <= t_interval(2));

t = t(idx);
p = p(idx);
v = v(idx);
u = u(idx);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HLIP Theoretical

% continuous dybnamics of the HLIP
lam = sqrt(g/z0_des);
Kp_db = 1;
Kd_db = (1/lam) * coth(lam * T_SSP);
K = [Kp_db, Kd_db];
A_SSP = [0,     1;
         lam^2, 0];

% discrete S2S dynamics
exp_A_SSP_T_SSP = expm(A_SSP * T_SSP);
A_S2S = exp_A_SSP_T_SSP * [1, T_DSP;
                           0, 1];
B_S2S = exp_A_SSP_T_SSP * [-1;
                            0];
A_S2S_cl = (A_S2S + B_S2S * K);

% compute the desired HLIP preimpact
sigma_P1 = lam * coth(0.5 * lam * T_SSP);
p_minus_H = (v_des * T_tot) / (2 + T_DSP * sigma_P1);
v_minus_H = sigma_P1 * (v_des * T_tot) / (2 + T_DSP * sigma_P1);

% compute some phase plots
x_max = max([p; p_minus_H]);
x_min = min([p; p_minus_H]);
y_max = max([v; v_minus_H]);
y_min = min([v; v_minus_H]);
x_range = linspace(x_min, x_max, 25);
y_range = linspace(y_min, y_max, 25);
[X1, X2] = meshgrid(x_range, y_range);

% compute the vector fields at each point
vecotr_field = zeros(size(X1, 1), size(X1, 2), 2);
for i = 1:size(X1, 1)
    for j = 1:size(X1, 2)
        
        % get the state
        x = [X1(i, j);
             X2(i, j)];
        
        % compute the vecotr field
        vf = A_SSP * x;

        % normalize the vector field
        norm_vf = norm(vf);

        % store the vector field
        vecotr_field(i, j, :) = vf / norm_vf;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot all the individual ROM flows
figure(1);

% plot the results
subplot(3, 1, 1);
grid on; hold on;
yline(0);
plot(t, p, 'LineWidth', 2);
xlabel('t (s)');
ylabel('p (m)');
title('p = p_c - p_s');

subplot(3, 1, 2);
grid on; hold on;
yline(0);
plot(t, v, 'LineWidth', 2);
xlabel('t (s)');
ylabel('v (m/s)');
title('v');

subplot(3, 1, 3);
grid on; hold on;
yline(0);
plot(t, u, 'LineWidth', 2);
xlabel('t (s)');
ylabel('u (m)');
title('u');

% plot the continuous phase plot
figure(2);
grid on; hold on; axis equal;
xlabel('p (m)');
ylabel('v (m/s)');

% plot the vector fields
streamslice(X1, X2, vecotr_field(:, :, 1), vecotr_field(:, :, 2));

% end and start points
plot(p(1), v(1), 'go', 'MarkerSize', 10, 'LineWidth', 2); % start
plot(p(end), v(end), 'ro','MarkerSize', 10, 'LineWidth', 2); % end

% HLIP target point
plot(p_minus_H, v_minus_H, 'pentagram', 'MarkerSize', 10, 'LineWidth', 2); % target

% plot the actual trajectory
plot(p, v, 'k', 'LineWidth', 2);
