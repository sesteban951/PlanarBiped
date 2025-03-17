%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Just a function to find swing foot trajectories
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% swing foot parameters
h = 0.15;
p1 = 0.1;
p2 = 0.5;
y_offset = 0.0;

% find the circle parameters
[c, r, angle1, angle2] = circle_params(p1, p2, h);

% polar system parameters
params.kr = 5.0;
params.R = r;
params.c = c;
params.angle1 = angle1;
params.angle2 = angle2;
params.y_offset = y_offset;
params.T_SSP = 10.0;

% initial conditions
xr_0 = [1.0;    % initial r position
        pi/2];   % initial theta position
[px, py] = polar_to_cartesian(xr_0(1), xr_0(2));
xc_0 = [px; py]; % initial cartesian position

% time span
T_SSP = params.T_SSP;
tspan = 0: 0.025 : T_SSP;

% simulate
[t, xc] = ode45(@(t, x) cartesian_dyn(t, x, params), tspan, xc_0);

% create circle for visualization
theta = linspace(0, 2 * pi, 100);
x_circle = params.R .* cos(theta) + params.c(1);
y_circle = params.R .* sin(theta) + params.c(2) + y_offset;

% compute vector field grid
x_min = min([xc(:, 1)]) - 0.25;
x_max = max([xc(:, 1)]) + 0.25;
y_min = min([xc(:, 2)]) - 0.25;
y_max = max([xc(:, 2)]) + 0.25;
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

% plot the start and end points
cx = c(1);
cy = c(2);
plot(p1, y_offset, 'go', 'LineWidth', 3);
plot(p2, y_offset, 'ro', 'LineWidth', 3);
plot([cx, cx + r*cos(angle1)], [cy, cy + r*sin(angle1)] + y_offset, 'k--', 'LineWidth', 2);
plot([cx, cx + r*cos(angle2)], [cy, cy + r*sin(angle2)] + y_offset, 'k--', 'LineWidth', 2);

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

% compute the circle parameters
function [c, r,angle1, angle2] = circle_params(x1, x2, h)

    % find the circle center
    d = abs(x2 - x1) / 2;
    r = (h^2 + d^2) / (2*h);
    cx = (x1 + x2) / 2;
    if h <= d
        cy = h - r;
    else
        cy = r - h;
    end

    % first point hit from positive x-axis (CCW)
    if abs(cy) < 1e-6
        angle1 = pi/2;
        angle2 = pi/2;
    else
        angle = atan2(d, abs(cy));
        if x1 > cx
            angle1 = pi/2 - angle;
            angle2 = angle1 + 2*angle;
        else
            angle2 = pi/2 - angle;
            angle1 = angle2 + 2*angle;
        end
    end
    
    c = [cx, cy];
end

% cartesian dynamics
function xdot = cartesian_dyn(t, x, params)
    
    % unpack the parameters
    R = params.R;
    c = params.c;
    kr = params.kr;
    y_offset = params.y_offset;

    % unpack the cartesian states
    x1 = x(1) - c(1);
    x2 = x(2) - c(2) - y_offset;
    x_norm = sqrt(x1^2 + x2^2);

    % get the desired theta input
    u = get_input(t, x, params);

    % avoid division by zero
    if x_norm == 0
        x_norm = 1e-6;
    end

    % dynamics
    xdot = [-x2 * u - kr * (x1 / x_norm) * (x_norm - R);
             x1 * u - kr * (x2 / x_norm) * (x_norm - R)];
end

% get the polar input
function u = get_input(t, x, params)
    % fun sine wave
    % f = 0.2;
    % omega = 2 * pi * f;
    % u = pi * sin(omega * t);

    % track the trajectory
    c = params.c;
    theta = atan2(x(2) - c(1), x(1) - c(2));
    angle1 = params.angle1;
    angle2 = params.angle2;
    T = params.T_SSP;
    angle = ((angle2 - angle1)/T) * t + angle1;
    u = -5.0 * (theta - angle);
end

% function to convert from polar to cartesian coordinates
function [x1, x2] = polar_to_cartesian(r, theta)
    x1 = r * cos(theta);
    x2 = r * sin(theta);
end
