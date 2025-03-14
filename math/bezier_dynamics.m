%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Bezier Curve ODE Example
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;

% Define control points
P0 = [-2; 0];
P1 = [-1; 2];
P2 = [1; 2];
P3 = [2; 0];

% Define the ODE system
odefun = @(t, x) bezier_dyn(t, x, P0, P1, P2, P3);

% Time span for integration
t_min = 0;
t_max = 1;
dt = 1 / 30;
tspan = t_min : dt : t_max;

% Initial condition (starting point at t=0)
x0 = [P0; t_min];  % [x; y; t]

% Solve ODE using ode45
[t_sol, state_sol] = ode45(odefun, tspan, x0);

% Extract solutions
x_sol = state_sol(:,1);
y_sol = state_sol(:,2);

% Query the vector fields as a function of time
x_min = min([P0(1), P1(1), P2(1), P3(1)]) - 0.5;
x_max = max([P0(1), P1(1), P2(1), P3(1)]) + 0.5;
y_min = min([P0(2), P1(2), P2(2), P3(2)]) - 0.5;
y_max = max([P0(2), P1(2), P2(2), P3(2)]) + 0.5;
x_range = linspace(x_min, x_max, 50);
y_range = linspace(y_min, y_max, 50);
t_range = t_min : dt : t_max;

% Query the vector fields
vector_fields = zeros(length(x_range), length(y_range), 2, length(t_range));
for i = 1:length(x_range)
    for j = 1:length(y_range)
        for k = 1:length(t_range)
            % Compute the vector field at point (x, y) and time t
            xk = [x_range(i); y_range(j); t_range(k)];
            tk = t_range(k);

            % Compute the vector field using the Bézier dynamics function
            f_ = bezier_dyn(tk, xk, P0, P1, P2, P3);
            
            % Normalize the vector field
            f_ = f_(1:2);

            % Store the vector field
            vector_fields(i, j, :, k) = f_;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTTING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot the Bézier curve
figure;
hold on; grid on;
axis equal;

xline(0); yline(0);
plot(x_sol, y_sol, 'k--', 'LineWidth', 1.5);  % Bézier curve from ODE
plot([P0(1) P1(1) P2(1) P3(1)], [P0(2) P1(2) P2(2) P3(2)], 'ko--', 'LineWidth', 1.0); % Control points
xlabel('X'); ylabel('Y');

pause(0.5);
idx = 1;
tic;
while idx < length(t_range)

    % plot the vector field
    vf_t = vector_fields(:, :, :, idx);
    vf_x = vf_t(:, :, 1);
    vf_y = vf_t(:, :, 2);
    vf_quiver = quiver(x_range, y_range, vf_x, vf_y, 'r', 'LineWidth', 0.5);

    % plot the point
    pt = plot(x_sol(idx), y_sol(idx), 'b', 'Marker', 'o', 'MarkerSize', 10, 'LineWidth', 2);

    msg = sprintf('t = %.2f', t_range(idx));
    title(msg);

    drawnow;

    % wait until the next time step
    while toc < t_range(idx+1)
        % do nothing
    end

    % increment the index
    if idx >= length(t_range)
        break;
    else
        idx = idx + 1;
        if idx == length(t_range)
            % dont delete
        else
            % delete the previous plot
            delete(vf_quiver);
            delete(pt);
        end
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AUXILIARY FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Bezier Dynamics Function
function xdot = bezier_dyn(~, x, P0, P1, P2, P3)
    t = x(3);  % Extract time parameter t
    
    % Compute velocity from Bézier equation derivative
    dx = 3*(1 - t)^2 * (P1(1) - P0(1)) + 6*(1 - t)*t * (P2(1) - P1(1)) + 3*t^2 * (P3(1) - P2(1));
    dy = 3*(1 - t)^2 * (P1(2) - P0(2)) + 6*(1 - t)*t * (P2(2) - P1(2)) + 3*t^2 * (P3(2) - P2(2));
    
    % Time evolution
    dt = 1;
    
    % Output state derivative
    xdot = [dx; dy; dt];
end
