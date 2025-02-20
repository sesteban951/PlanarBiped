%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Planar Bipde Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% import the data
file_location = '../data/';
t = importdata(file_location + "time.csv");
pos = importdata(file_location + "pos.csv");
vel = importdata(file_location + "vel.csv");
tau = importdata(file_location + "tau.csv");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  what to plot
plot_states = 1;
plot_orbits = 0;
orbit_T = 1.0;

% time interval
time_segment = 1;
t_interval = [t(1), t(end)];
% t_interval = [0, 3];

% downsampling
down_sample = 0;
hz = 60;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% segement based on time
if time_segment == 1
    idx = find(t >= t_interval(1) & t <= t_interval(2));

    t = t(idx);
    pos = pos(idx,:);
    vel = vel(idx,:);
    tau = tau(idx,:);
end

% downsample data
dt = t(2) - t(1);
if down_sample == 1
    nth_sample = round(1 / (dt * hz));
    idx = 1:nth_sample:length(t);

    t = t(idx);
    pos = pos(idx,:);
    vel = vel(idx,:);
    tau = tau(idx,:);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% positions
px_base = pos(:,1);
pz_base = pos(:,2);
theta_base = pos(:,3);
q_HL = pos(:,4);
q_KL = pos(:,5);
q_HR = pos(:,6);
q_KR = pos(:,7);

% velocities
vx_base = vel(:,1);
vz_base = vel(:,2);
thetadot_base = vel(:,3);
qd_HL = vel(:,4);
qd_KL = vel(:,5);
qd_HR = vel(:,6);
qd_KR = vel(:,7);

% torques
tau_HL = tau(:,1);
tau_KL = tau(:,2);
tau_HR = tau(:,3);
tau_KR = tau(:,4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for plotting the state
if plot_states == 1

    fig = figure('Name', 'Planar Biped Data');
    tabgroup = uitabgroup(fig);

    % POSITION
    tab_pos = uitab(tabgroup, 'Title', 'Position');
    axes_pos = axes('Parent', tab_pos);

    % Base
    subplot(3,3,1);
    hold on; yline(0);
    plot(t, px_base);
    title('Base px');
    xlabel('Time (s)');
    grid on;

    subplot(3,3,4);
    hold on; yline(0);
    plot(t, pz_base);
    title('Base pz');
    xlabel('Time (s)');
    grid on;

    subplot(3,3,7);
    hold on; yline(0);
    plot(t, theta_base);
    title('Base theta');
    xlabel('Time (s)');
    grid on;

    % Left Leg
    subplot(3,3,2);
    hold on; yline(0);
    plot(t, q_HL);
    title('Left q_H');
    xlabel('Time (s)');
    grid on;

    subplot(3,3,5);
    hold on; yline(0);
    plot(t, q_KL);
    title('Left q_K');
    xlabel('Time (s)');
    grid on;

    % Right Leg
    subplot(3,3,3);
    hold on; yline(0);
    plot(t, q_HR);
    title('Right q_H');
    xlabel('Time (s)');
    grid on;

    subplot(3,3,6);
    hold on; yline(0);
    plot(t, q_KR);
    title('Right q_K');
    xlabel('Time (s)');
    grid on;

    % VELOCITY
    tab_vel = uitab(tabgroup, 'Title', 'Velocity');
    axes_vel = axes('Parent', tab_vel);

    % Base
    subplot(3,3,1);
    hold on; yline(0);
    plot(t, vx_base);
    title('Base vx');
    xlabel('Time (s)');
    grid on;

    subplot(3,3,4);
    hold on; yline(0);
    plot(t, vz_base);
    title('Base vz');
    xlabel('Time (s)');
    grid on;

    subplot(3,3,7);
    hold on; yline(0);
    plot(t, thetadot_base);
    title('Base thetadot');
    xlabel('Time (s)');
    grid on;

    % Left Leg
    subplot(3,3,2);
    hold on; yline(0);
    plot(t, qd_HL);
    title('Left qd_H');
    xlabel('Time (s)');
    grid on;

    subplot(3,3,5);
    hold on; yline(0);
    plot(t, qd_KL);
    title('Left qd_K');
    xlabel('Time (s)');
    grid on;

    % Right Leg
    subplot(3,3,3);
    hold on; yline(0);
    plot(t, qd_HR);
    title('Right qd_H');
    xlabel('Time (s)');
    grid on;

    subplot(3,3,6);
    hold on; yline(0);
    plot(t, qd_KR);
    title('Right qd_K');
    xlabel('Time (s)');
    grid on;

    % TORQUE
    tab_tau = uitab(tabgroup, 'Title', 'Torque');
    axes_tau = axes('Parent', tab_tau);

    % Left Leg
    subplot(2,2,1);
    hold on; yline(0);
    plot(t, tau_HL);
    title('Left tau_H');
    xlabel('Time (s)');
    grid on;

    subplot(2,2,2);
    hold on; yline(0);
    plot(t, tau_KL);
    title('Left tau_K');
    xlabel('Time (s)');
    grid on;

    % Right Leg
    subplot(2,2,3);
    hold on; yline(0);
    plot(t, tau_HR);
    title('Right tau_H');
    xlabel('Time (s)');
    grid on;

    subplot(2,2,4);
    hold on; yline(0);
    plot(t, tau_KR);
    title('Right tau_K');
    xlabel('Time (s)');
    grid on;


    % ORBITS
    tab_orbit = uitab(tabgroup, 'Title', 'Orbit');
    axes_orbit = axes('Parent', tab_orbit);

    subplot(2,2,1);
    hold on; xline(0); yline(0);
    plot(q_HL, qd_HL);
    title('Left Hip');
    xlabel('q');
    ylabel('qd');
    grid on;

    subplot(2,2,2);
    hold on; xline(0); yline(0);
    plot(q_KL, qd_KL);
    title('Left Knee');
    xlabel('q');
    ylabel('qd');
    grid on;

    subplot(2,2,3);
    hold on; xline(0); yline(0);
    plot(q_HR, qd_HR);
    title('Right Hip');
    xlabel('q');
    ylabel('qd');
    grid on;

    subplot(2,2,4);
    hold on; xline(0); yline(0);
    plot(q_KR, qd_KR);
    title('Right Knee');
    xlabel('q');
    ylabel('qd');
    grid on;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if plot_orbits == 1
    figure('Name', 'Phase Plots');

    % parse data;
    q_HL_min = min(q_HL);
    q_HL_max = max(q_HL);
    qd_HL_min = min(qd_HL);
    qd_HL_max = max(qd_HL);

    % make the plot
    plot(nan, nan);
    hold on; grid on;
    xline(0); yline(0);
    xlabel('q');
    ylabel('qdot');

    xlim([q_HL_min  - 0.2, q_HL_max  + 0.2]);
    ylim([qd_HL_min - 0.2, qd_HL_max + 0.2]);

    % initialize the orbit
    dt = t(2) - t(1);
    N_orbit = round(orbit_T / dt);
    orbit = nan(2, N_orbit);

    % Initialize the orbit plot
    orbit_plot = plot(nan, nan, 'b', 'LineWidth', 2);

    idx = 1;
    tic;
    while idx< length(t)
        
        % plot the data
        x_HL = [q_HL(idx); qd_HL(idx)];
        pt = plot(x_HL(1), x_HL(2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');

        % Store new data and remove the oldest
        orbit(:, 1:end-1) = orbit(:, 2:end); % Shift left
        orbit(:, end) = [q_HL(idx); qd_HL(idx)]; % Add new data

        % Clear previous orbit
        if exist('orbit_plots', 'var')
            delete(orbit_plots);
        end

        % Plot the orbit as fading lines
        hold on;
        num_segments = size(orbit, 2) - 1; % Number of line segments
        cmap = linspace(1, 0, num_segments)'; % Create a fading colormap (1 = full color, 0 = white)
        orbit_plots = gobjects(1, num_segments); % Store handles to delete later
        for i = 1:num_segments
            color = [1, cmap(i), cmap(i)]; % Red with fading effect
            orbit_plots(i) = plot(orbit(1, i:i+1), orbit(2, i:i+1), '-', ...
                'LineWidth', 2, 'Color', color);
        end

        % update the index
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
                % dont delte
            else
                delete(pt);
            end
        end

    end
end
