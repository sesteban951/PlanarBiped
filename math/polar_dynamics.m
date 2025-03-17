%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Just a function to find swing foot trajectories
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% polar system parameters
params.R = 1.0;
params.kr = 1.0;

% initial conditions
xr_0 = [1.5;  % initial r position
        0];  % initial theta position
xc_0 = [1.5;  % initial x position
        0]; % initial y position

% time span
tspan = 0:0.025:10;

% simulate
[~, xr] = ode45(@(t, x) polar_dyn(t, x, params), tspan, xr_0);
[t, xc] = ode45(@(t, x) cartesian_dyn(t, x, params), tspan, xc_0);

% convert the polar coordinates to cartesian
xr_converted = zeros(length(t), 2);
for i = 1:length(t)
    [xr_1, xr_2] = polar_to_cartesian(xr(i, 1), xr(i, 2));
    xr_converted(i, :) = [xr_1, xr_2];
end

% create circle for visualization
theta = linspace(0, 2 * pi, 100);
x_circle = params.R .* cos(theta);
y_circle = params.R .* sin(theta);

% compute vector field grid
x_min = min([xr_converted(:, 1); xc(:, 1)]) - 0.25;
x_max = max([xr_converted(:, 1); xc(:, 1)]) + 0.25;
y_min = min([xr_converted(:, 2); xc(:, 2)]) - 0.25;
y_max = max([xr_converted(:, 2); xc(:, 2)]) + 0.25;
x1_range = linspace(x_min, x_max, 25);
x2_range = linspace(y_min, y_max, 25);
[X, Y] = meshgrid(x1_range, x2_range);

% compute the vector fields
vector_fields = zeros(length(x1_range), length(x2_range), 2, length(t));
for i = 1:length(t)
    for j = 1:length(x1_range)
        for k = 1:length(x2_range)
            
            % get the current state
            x = [X(k, j); Y(k, j)];

            % compute the dynamics
            vector_fields(j, k, :, i) = cartesian_dyn(t(i), x, params);
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot the solution
figure(1);
hold on; grid on; axis equal
xline(0); yline(0);
xlim([x_min, x_max]);
ylim([y_min, y_max]);

% plot the circle
plot(x_circle, y_circle, 'k--', 'LineWidth', 2);
xlabel('x1');
ylabel('x2');

% plot the vector field animation
pause(0.5);
idx = 1;
tic;
while idx < length(t)

    % extract vector field at time idx
    vf_t = vector_fields(:, :, :, idx);
    vf_x1 = squeeze(vf_t(:, :, 1))';
    vf_x2 = squeeze(vf_t(:, :, 2))';

    % normalize vectors for better visualization
    vf_mag = sqrt(vf_x1.^2 + vf_x2.^2);
    vf_x1 = vf_x1 ./ (vf_mag + 1e-3);
    vf_x2 = vf_x2 ./ (vf_mag + 1e-3);

    % plot the vector field
    vf_quiver = quiver(X, Y, vf_x1, vf_x2, 'r', 'LineWidth', 0.5);

    % plot the solution trajectory
    pt_trail = plot(xc(idx, 1), xc(idx, 2), 'b.', 'MarkerSize', 10, 'LineWidth', 2);
    pt = plot(xc(idx, 1), xc(idx, 2), 'k', 'Marker', 'o', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'b');

    msg = sprintf('t = %.2f s', t(idx));
    title(msg);

    % update plot
    drawnow;

    % wait until the next time step
    while toc < t(idx+1)
        % do nothing
    end

    % increment the index
    if idx >= length(t)
        break;
    else
        idx = idx + 1;
        if idx < length(t)
            delete(vf_quiver);
            delete(pt);
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% polar dynamics
function xdot = polar_dyn(t, x, params)
    
    % unpack the parameters
    R = params.R;
    kr = params.kr;

    % unpack the polar states
    r = x(1);
    theta = x(2);

    % get the desired theta input
    u = get_input(t, theta);

    % dynamics
    xdot = [-kr * (r - R);
            u];
end

% cartesian dynamics
function xdot = cartesian_dyn(t, x, params)
    
    % unpack the parameters
    R = params.R;
    kr = params.kr;

    % unpack the cartesian states
    x1 = x(1);
    x2 = x(2);
    x_norm = sqrt(x1^2 + x2^2);

    % get the desired theta input
    u = get_input(t, x);

    % avoid division by zero
    if x_norm == 0
        x_norm = 1e-6;
    end

    % dynamics
    xdot = [-x2 * u - kr * (x1 / x_norm) * (x_norm - R);
             x1 * u - kr * (x2 / x_norm) * (x_norm - R)];
end

% get the polar input
function u = get_input(t, x)
    f = 0.2;
    omega = 2 * pi * f;
    u = pi * sin(omega * t);
end

% function to convert from polar to cartesian coordinates
function [x1, x2] = polar_to_cartesian(r, theta)
    x1 = r * cos(theta);
    x2 = r * sin(theta);
end
