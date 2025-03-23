%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Planar Bipde Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% HLIP dynamics
g = 9.81;
z0 = 1.0;
lam = sqrt(g/z0);
T_SSP = 0.5;
T_DSP = 0.0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% continuous phase dynamics
A_SSP = [0,     1;
         lam^2, 0];

% discrete S2S dynamics
exp_A_SSP_T_SSP = expm(A_SSP * T_SSP);
A_S2S = exp_A_SSP_T_SSP * [1, T_DSP;
                           0, 1];
B_S2S = exp_A_SSP_T_SSP * [-1;
                            0];

% LQR controller
Q_x = diag([1, 0.1]);
Q_u = 1;
K_lqr = -lqrd(A_S2S, B_S2S, Q_x, Q_u, T_SSP);

% deadbeat controller
K_db = [1, T_DSP + (1/lam) * coth(T_SSP * lam)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% choose the linear control law you want
% K = K_db;
K = K_lqr;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Lyapunov equation
Q = eye(2);
A_cl = (A_S2S + B_S2S * K);
P = dlyap(A_cl, Q);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initial condition
x0 = [0.2;
      -0.2];

% simulate forward in time
n_steps = 6;
x_t = zeros(2, n_steps);
u_t = zeros(1, n_steps-1);
V_t = zeros(1, n_steps);
x_t(:, 1) = x0;

for i = 1:n_steps-1

    % compute the control input
    xk = x_t(:, i);
    uk = K * xk;

    % compute the Lyapunov function
    V_t(i) = xk' * P * xk;

    % forward simulate
    u_t(:, i) = uk;
    x_t(:, i+1) = A_S2S * xk + B_S2S * uk;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% compute the HLIP gradient fields
max_val = max(abs(x_t(:)));
x_range = linspace(-max_val - 0.1, max_val + 0.1, 25);
y_range = linspace(-max_val - 0.1, max_val + 0.1, 25);
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

% compute the continuous phase dynamics
dt = 0.01;
t_continuous = 0:dt:T_SSP;
x_t_continuous = zeros(length(t_continuous), 2, n_steps);
for i = 1:n_steps
    
    % initial condition 
    x0 = x_t(:, i);

    % compute the continuous phase dynamics
    for j = 1:length(t_continuous)
        e_ASSP = expm(A_SSP * t_continuous(j));
        x_t_continuous(j, :, i) = (e_ASSP * x0);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% plot the results
figure(1);

xlims = [min(x_t(1, :)) - 0.1, max(x_t(1, :)) + 0.1];
ylims = [min(x_t(2, :)) - 0.1, max(x_t(2, :)) + 0.1];

% plot the discrete dynamics
subplot(2, 2, 1);
grid on; hold on;
xlim(xlims);
ylim(ylims);
xline(0); yline(0);
plot(x_t(1, :), x_t(2, :), 'o-','LineWidth', 2); % trajectory
plot(x_t(1, 1), x_t(2, 1), 'gx','LineWidth', 2); % start
plot(x_t(1, end), x_t(2, end), 'rx','LineWidth', 2); % end
xlabel('p (m)');
ylabel('v (m/s)');
title('Dsicrete Dynamics');

% plot the discrete inputs
subplot(2, 2, 3);
grid on; hold on;
yline(0);
stairs(1:n_steps-1, u_t(:), 'o-','LineWidth', 2); % input
xlabel('k');
ylabel('u (m)');
xticks(1:n_steps-1);
title('Control Input');

% plot the continuous flows
subplot(2, 2, 2);
grid on; hold on;
xlim([x_range(1), x_range(end)]);
ylim([y_range(1), y_range(end)]);
xline(0); yline(0);
quiver(X1, X2, vecotr_field(:, :, 1), vecotr_field(:, :, 2), 'r', 'LineWidth', 0.5);
for i = 1:n_steps
    plot(x_t_continuous(:, 1, i), x_t_continuous(:, 2, i), 'k-','LineWidth', 2); % trajectory
end
plot(x_t(1, :), x_t(2, :), 'ko','LineWidth', 2); % trajectory

% plot the lyapunov function
subplot(2 ,2, 4);
grid on; hold on; axis equal;
x1_max = max(abs(x_t(1, :)));
x2_max = max(abs(x_t(2, :)));
x_max = max(abs([x1_max, x2_max]));

x1_range = linspace(-x_max - 0.1, x_max + 0.1, 50);
x2_range = linspace(-x_max - 0.1, x_max + 0.1, 50);
[X1, X2] = meshgrid(x1_range, x2_range);
V = P(1,1)*X1.^2 + 2*P(1,2)*X1.*X2 + P(2,2)*X2.^2;

% plot the x, y, z axis
quiver3(0, 0, 0, 1, 0, 0, 'r');
quiver3(0, 0, 0, 0, 1, 0, 'g');
quiver3(0, 0, 0, 0, 0, 1, 'b');

surf(X1, X2, V, 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'FaceColor', [0.5, 0.5, 0.5]); % Gray color
plot3(x_t(1, :), x_t(2, :), V_t, 'ko','LineWidth', 2); % trajectory
xlabel('x_1');
ylabel('x_2');
zlabel('V(x)');
title('Quadratic Function V = x^T P x');
view(3);
grid on;

