%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Just a function to find swing foot tajectories
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% desired circle parameters
h = 0.1;
x1 = -0.1;
x2 = 0.1;
y_offset = 0.0;

% find the circle center
cx = (x1 + x2) / 2;
r = (h^2 + (x2 - x1)^2 / 4) / (2*h);
cy = y_offset + h - r;

if cy >0
    cy = -cy;
end

% find the circle points
theta = linspace(0, 2*pi, 100);
x = cx + r * cos(theta);
y = cy + r * sin(theta);

% populate the circle parameters
params.R = r;
params.xc = cx;
params.yc = cy;
params.gamma = 15.0;

% generate a vector field around the circle
x_max = cx + r + 0.1;
x_min = cx - r - 0.1;
y_max = cy + r + 0.1;
y_min = cy - r - 0.1;
x_range = linspace(x_min, x_max, 50);
y_range = linspace(y_min, y_max, 50);
[X1, X2] = meshgrid(x_range, y_range);

% compute the vector field
vector_field = zeros(length(X1), length(X2), 2);
for i = 1:length(X1)
    for j = 1:length(X2)

        % compute the vector field at point (x1, x2)
        xk = [X1(i, j); X2(i, j)];
        f_ = f(xk, params);

        % normalize the vector field
        f_ = f_ / norm(f_);

        % store the vector field
        vector_field(i, j, :) = f_;
    end
end

% simualte the dynamics
x0 = [0.1,0.1];
tmax = 15;
dt = 1/30;
tspan = 0 : dt : tmax;
[t, x_t] = ode45(@(t,x) f(x, params), tspan, x0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTTING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1);
hold on; grid on;
axis equal;

% plot the vector field
% quiver(X1, X2, vector_field(:, :, 1), vector_field(:, :, 2), 'k', 'LineWidth', 0.5);
streamslice(X1, X2, vector_field(:, :, 1), vector_field(:, :, 2));

% plot the circle
xline(0); yline(0);
plot(x, y, 'k', 'LineWidth', 2);
plot(cx, cy, 'ko', 'LineWidth', 3);

% plot the start and end points
plot(x1, y_offset, 'ro', 'LineWidth', 3);
plot(x2, y_offset, 'ro', 'LineWidth', 3);

xlabel('x (m)');
ylabel('y (m)');

% plot the solution
% plot(x_t(:, 1), x_t(:, 2), 'm', 'LineWidth', 2);
pause(0.25);
idx = 1;
tic;
while idx < length(t)

    % plot the point
    pt = plot(x_t(idx, 1), x_t(idx, 2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);

    msg = sprintf('t = %.2f', t(idx));
    title(msg);

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
        if idx == length(t)
            % dont delete
        else
            % delete the previous plot
            delete(pt);
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AUXILIARY FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% circular periodic orbit dynamics
function f = f(x, params)

    % unpack circle parameters
    yc = params.yc;
    xc = params.xc;
    R = params.R;
    gamma = params.gamma;

    % compute the dynamics
    f = [(x(2) - yc) + gamma * (x(1) - xc) * (R^2 - (x(1) - xc)^2 - (x(2) - yc)^2);
        -(x(1) - xc) + gamma * (x(2) - yc) * (R^2 - (x(1) - xc)^2 - (x(2) - yc)^2)];
end
