%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot ROM Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; clc; close all;

% import the data
file_location = 'cpp/logs/';
% t = importdata(file_location + "time.csv");

% % % unpack the output data
% x = importdata(file_location + "rom_state.csv");
% u = importdata(file_location + "rom_input.csv");

% p = x(:,1);
% v = x(:,2);

data = readtable(file_location + "log.csv");
p = data.y_1;
v = data.y_dot_1;
t = data.t;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % extract some data from the yaml config file
% config_file_path = "../config/biped.yaml";
% config = yaml.loadFile(config_file_path);

% some parameters
g = 9.81;
% z0_des = config.HLIP.z0;
z0_des = 0.85;
% v_des = config.HLIP.v_des;
v_des = 1.00;

% The total mass of the robot
mass = 10.426;

l_thigh = 0.5;
l_shin = 0.5;

% T_SSP = config.HLIP.T_SSP;
T_SSP = 0.3;
T_DSP = 0.0;
T_tot = T_SSP + T_DSP;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROBOT Emperical

% time window of interest
% t_interval = [t(1), t(end)];
t_interval = [t(end) - 3, t(end)];
idx = find(t >= t_interval(1) & t <= t_interval(2));

t = t(idx);
p = p(idx);
v = v(idx);
post_impact = data.post_impact;
% u = u(idx);

post_impact_indices = find(post_impact == 1);
pre_impact_indices = post_impact_indices - 1;

Q_1_post_impact = data.z_1(post_impact_indices);
Q_1_pre_impact = data.z_1(pre_impact_indices);
L_post_impact = data.z_1_dot(post_impact_indices);
L_pre_impact = data.z_1_dot(pre_impact_indices);

q_2_pos = data.q_2(idx);
q_3_pos = data.q_3(idx);
q_4_pos = data.q_4(idx);
q_5_pos = data.q_5(idx);

q_2_vel = data.q_2_dot(idx);
q_3_vel = data.q_3_dot(idx);
q_4_vel = data.q_4_dot(idx);
q_5_vel = data.q_5_dot(idx);

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

% Compute the HLIP trajectory
[P, V] = GetHLIPPhaseTrajectory(z0_des, g, v_des, T_SSP, T_DSP);

% Convert the HLIP trajectory into zero dynamics
[Q_1, Q_2, L] = ConvertHLIPToZeroDynamics(P, V, z0_des, l_thigh, l_shin, mass);

% [Q_1_post_impact, Q_2_post_impact, L_post_impact] = ConvertHLIPToZeroDynamics(p_post_impact, v_post_impact, z0_des, l_thigh, l_shin, mass);
% [Q_1_pre_impact, Q_2_pre_impact, L_pre_impact] = ConvertHLIPToZeroDynamics(p_pre_impact, v_pre_impact, z0_des, l_thigh, l_shin, mass);

[Q_1_dot, Q_2_dot, L_dot] = SolveHLIPVelIK(P, V, Q_1, Q_2, mass, g, l_thigh, l_shin);

% compute some phase plots
q1_max = 0.75;
q1_min = 0.25;
L_max = 9.8;
L_min = 8.4;
x_range = linspace(q1_min, q1_max, 20);
y_range = linspace(L_min, L_max, 20);
[X1, X2] = meshgrid(x_range, y_range);

% compute the vector fields at each point
vector_field = zeros(size(X1, 1), size(X1, 2), 2);
for i = 1:size(X1, 1)
    for j = 1:size(X1, 2)
        
        % get the state
        x = [X1(i, j);
             X2(i, j)];
        
        % compute the vecotr field
        q_1 = x(1);
        l = x(2);
        [p_hlip, v_hlip, q_2] = GetHLIPOutputs(q_1, l, l_thigh, l_shin, mass, z0_des);
        %[q_1, q_2, l] = ConvertHLIPToZeroDynamics(p_hlip, v_hlip, z0_des, l_thigh, l_shin, mass);
        [q_1_dot, q_2_dot, l_dot] = SolveHLIPVelIK(p_hlip, v_hlip, q_1, q_2, mass, g, l_thigh, l_shin);

        % Get the derivative
        vf = [q_1_dot; l_dot];

        % normalize the vector field
        norm_vf = norm(vf);

        % store the vector field
        vector_field(i, j, :) = vf / norm_vf;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ==== Parameters ====
fig_width = 1920;       % Figure width
fig_height = 1080;      % Figure height
line_width = 5;         % Line width for plots 
marker_size = 50;        % Marker size
marker_line_width = 5;
font_size = 18;         % Font size for labels
legend_font_size = 40;  % Font size for legend
title_font_size = 40;   % Font size for title
arrow_scale = 3;      % Scaling factor for stream arrows
tick_font_size = 40;    % Font size for tick labels
tick_length = [0.02, 0.02];  % Tick length (normalized units)

% ==== Create Figure ====
figure(1);
hold on;

% ==== Plot Trajectories ====
plot(data.z_1(idx), data.z_1_dot(idx), 'LineWidth', line_width, 'Color', 'b');
plot(Q_1, L, 'LineWidth', line_width, 'Color', 'r');
plot(Q_1_pre_impact(end), L_pre_impact(end), '*', 'MarkerSize', marker_size, 'LineWidth', marker_line_width, 'Color', [0 1 1]);
plot(Q_1(end-1), L(end-1), '*', 'MarkerSize', marker_size, 'LineWidth', marker_line_width, 'Color', [1 0 1]);

% % Adjust the scaling of the streamlines
% h = streamslice(X1, X2, vector_field(:, :, 1), vector_field(:, :, 2), arrow_scale);
% 
% % Set the color and line thickness of each streamline (arrow)
% for i = 1:length(h)
%     set(h(i), 'Color', [0 0 0], 'LineWidth', 2);  % Set the color to black and thickness to 2
% end

% Define scaling factor for arrow lengths
arrow_scale = 0.5;  % Adjust as needed

% Plot vector field using quiver
h = quiver(X1, X2, vector_field(:, :, 1), vector_field(:, :, 2), arrow_scale, 'k', 'LineWidth', 2);

% ==== Plot Trajectories ====
plot(data.z_1(idx), data.z_1_dot(idx), 'LineWidth', line_width, 'Color', 'b');
plot(Q_1, L, 'LineWidth', line_width, 'Color', 'r');
plot(Q_1_pre_impact(end), L_pre_impact(end), '*', 'MarkerSize', marker_size, 'LineWidth', marker_line_width, 'Color', [0 1 1]);
plot(Q_1(end-1), L(end-1), '*', 'MarkerSize', marker_size, 'LineWidth', 5, 'Color', [1 0 1]);

% ==== Set Labels with LaTeX ====
xlabel('$z_1$', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('${z}_2$', 'Interpreter', 'latex', 'FontSize', font_size);

% ==== Set Tick Sizes and LaTeX Ticks ====
ax = gca;  % Get current axis
ax.FontSize = tick_font_size;  % Adjust tick font size
ax.TickLength = tick_length;   % Adjust tick length

% Convert tick labels to LaTeX-style
% ax.XTickLabel = arrayfun(@(x) sprintf('$%.1f$', x), ax.XTick, 'UniformOutput', false);
% ax.YTickLabel = arrayfun(@(y) sprintf('$%.1f$', y), ax.YTick, 'UniformOutput', false);

% Force LaTeX interpreter for ticks
set(gca, 'TickLabelInterpreter', 'latex');

% ==== Set Figure Size ====
set(gcf, 'Position', [0, 0, fig_width, fig_height]);

% ==== Add Legend ====
%legend("$z^{*}$", "$\Xi^*$", "$\mathbf{f}(\mathbf{r})$", 'Interpreter', 'latex', 'FontSize', legend_font_size, 'Location', 'Southeast');
legend("$\mathcal{O}_\mathbf{z}$", "$\mathcal{O}_{\Xi(\mathbf{r})}$", "$\mathcal{O}_\mathbf{z}^*$", "$\dot{\Xi}(\mathbf{r})$", 'Interpreter', 'latex', 'FontSize', legend_font_size, 'Location', 'Southeast');

% ==== Add Title ====
title('\textbf{Phase Portrait with Vector Field}', 'Interpreter', 'latex', 'FontSize', title_font_size);

% ==== Final Adjustments ====
grid on;
hold off;

xlim([q1_min, q1_max]);
ylim([L_min, L_max]);

exportgraphics(gcf, 'phase_plot.eps', 'ContentType', 'vector', 'BackgroundColor', 'none', 'Resolution', 300);
exportgraphics(gcf, 'phase_plot.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none', 'Resolution', 300);



%% Actuated coordinates
figure(2);

n_rows = 2;
n_cols = 2;
n = 0;

n = n + 1;
subplot(n_rows, n_cols, n);
plot(q_2_pos, q_2_vel, 'LineWidth', line_width, 'Color', 'b');
xlabel('$q_2$', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$\dot{q}_2$', 'Interpreter', 'latex', 'FontSize', font_size);
title('Phase portrait (${q}_2$)', 'Interpreter', 'latex', 'FontSize', title_font_size);

% ==== Set Tick Sizes and LaTeX Ticks ====
ax = gca;  % Get current axis
ax.FontSize = tick_font_size;  % Adjust tick font size
ax.TickLength = tick_length;   % Adjust tick length

n = n + 1;
subplot(n_rows, n_cols, n);
plot(q_3_pos, q_3_vel, 'LineWidth', line_width, 'Color', 'b');
xlabel('$q_3$', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$\dot{q}_3$', 'Interpreter', 'latex', 'FontSize', font_size);
title('Phase portrait (${q}_3$)', 'Interpreter', 'latex', 'FontSize', title_font_size);

% ==== Set Tick Sizes and LaTeX Ticks ====
ax = gca;  % Get current axis
ax.FontSize = tick_font_size;  % Adjust tick font size
ax.TickLength = tick_length;   % Adjust tick length

n = n + 1;
subplot(n_rows, n_cols, n);
plot(q_4_pos, q_4_vel, 'LineWidth', line_width, 'Color', 'b');
xlabel('$q_4$', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$\dot{q}_4$', 'Interpreter', 'latex', 'FontSize', font_size);
title('Phase portrait (${q}_4$)', 'Interpreter', 'latex', 'FontSize', title_font_size);

% ==== Set Tick Sizes and LaTeX Ticks ====
ax = gca;  % Get current axis
ax.FontSize = tick_font_size;  % Adjust tick font size
ax.TickLength = tick_length;   % Adjust tick length

n = n + 1;
subplot(n_rows, n_cols, n);
plot(q_5_pos, q_5_vel, 'LineWidth', line_width, 'Color', 'b');
xlabel('$q_5$', 'Interpreter', 'latex', 'FontSize', font_size);
ylabel('$\dot{q}_5$', 'Interpreter', 'latex', 'FontSize', font_size);
title('Phase portrait (${q}_5$)', 'Interpreter', 'latex', 'FontSize', title_font_size);

% ==== Set Tick Sizes and LaTeX Ticks ====
ax = gca;  % Get current axis
ax.FontSize = tick_font_size;  % Adjust tick font size
ax.TickLength = tick_length;   % Adjust tick length

% Force LaTeX interpreter for ticks
set(gca, 'TickLabelInterpreter', 'latex');

% ==== Set Figure Size ====
set(gcf, 'Position', [0, 0, fig_width, fig_height]);

% Export the figure
exportgraphics(gcf, 'phase_plots_actuated.eps', 'ContentType', 'vector', 'BackgroundColor', 'none', 'Resolution', 300);
exportgraphics(gcf, 'phase_plots_actuated.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none', 'Resolution', 300);


% % plot all the individual ROM flows
% figure(1);
% 
% % plot the results
% subplot(2, 1, 1);
% grid on; hold on;
% yline(0);
% plot(t, p, 'LineWidth', 2);
% xlabel('t (s)');
% ylabel('p (m)');
% title('p = p_c - p_s');
% 
% subplot(2, 1, 2);
% grid on; hold on;
% yline(0);
% % plot(t, v, 'LineWidth', 2);
% plot(t, v, 'b.');
% xlabel('t (s)');
% ylabel('v (m/s)');
% title('v');
% 
% set(gcf, 'Position', [961, -200, 960, 450]);
% 
% % subplot(3, 1, 3);
% % grid on; hold on;
% % yline(0);
% % % plot(t, u, 'LineWidth', 2);
% % xlabel('t (s)');
% % ylabel('u (m)');
% % title('u');
% 
% % plot the continuous phase plot
% figure(2);
% grid on; hold on; axis equal;
% xlabel('p (m)');
% ylabel('v (m/s)');
% 
% % plot the vector fields
% %streamslice(X1, X2, vector_field(:, :, 1), vector_field(:, :, 2));
% 
% % Plot the HLIP trajectory
% plot(P, V, 'r', 'Linewidth', 2);
% 
% % end and start points
% plot(p(1), v(1), 'go', 'MarkerSize', 10, 'LineWidth', 2); % start
% plot(p(end), v(end), 'ro','MarkerSize', 10, 'LineWidth', 2); % end
% plot(p_pre_impact, v_pre_impact, '*');
% plot(p_post_impact, v_post_impact, 'o');
% 
% % HLIP target point
% plot(p_minus_H, v_minus_H, 'pentagram', 'MarkerSize', 10, 'LineWidth', 2); % target
% 
% % plot the actual trajectory
% plot(p, v, 'k', 'LineWidth', 2);
% 
% set(gcf, 'Position', [961, 900, 960, 450]);


% Plot the zero dynamics states
% 
% figure(3);
% 
% n_rows = 4;
% n_cols = 1;
% n = 0;
% 
% % z1
% n = n + 1;
% subplot(n_rows, n_cols, n);
% hold on;
% grid on;
% plot(data.t(idx), data.z_1(idx));
% title('$z$', 'Interpreter', 'latex');
% xlabel('$t$', 'Interpreter', 'latex');
% ylabel('$z$', 'Interpreter', 'latex');
% 
% % z_dot
% n = n + 1;
% subplot(n_rows, n_cols, n);
% hold on;
% grid on;
% plot(data.t(idx), data.z_1_dot(idx));
% title('$\dot{z}$', 'Interpreter', 'latex');
% xlabel('$t$', 'Interpreter', 'latex');
% ylabel('$\dot{z}$', 'Interpreter', 'latex');
% 
% set(gcf, 'Position', [0, -200, 960, 450]);
% 
% %% Phase portrait
% figure(4);
% 
% hold on;
% plot(data.z_1(idx), data.z_1_dot(idx));
% plot(Q_1, L);
% xlabel('$z$', 'Interpreter', 'latex');
% ylabel('$\dot{z}$', 'Interpreter', 'latex');
% set(gcf, 'Position', [0, 900, 960, 450]);
% 
% %% Plot the zero dynamics states
% figure(5);
% 
% n_rows = 4;
% n_cols = 1;
% n = 0;
% 
% % z1
% n = n + 1;
% subplot(n_rows, n_cols, n);
% hold on;
% grid on;
% plot(data.t(idx), data.y_1(idx), 'b.');
% title('$p$', 'Interpreter', 'latex');
% xlabel('$t$', 'Interpreter', 'latex');
% ylabel('$p$', 'Interpreter', 'latex');
% 
% % z_dot
% n = n + 1;
% subplot(n_rows, n_cols, n);
% hold on;
% grid on;
% plot(data.t(idx), data.y_dot_1(idx), 'b.');
% title('$\dot{v}$', 'Interpreter', 'latex');
% xlabel('$t$', 'Interpreter', 'latex');
% ylabel('$\dot{v}$', 'Interpreter', 'latex');
% 
% set(gcf, 'Position', [0, -200, 960, 450]);
