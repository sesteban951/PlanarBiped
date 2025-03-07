%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lower to higher dimensional integrator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;

% parameters
params.k_single = 0.5;
params.kd_double = 5.0;

% initial conditions
z0_single =  2;  % p
x0_double = [2;  % p 
             0]; % v
x0 = [z0_single; x0_double];

% simulation 
t_sim = 10.0;
dt = 0.01;
t = 0:dt:t_sim;

% initialize the state
[t, x] = ode45(@(t, x) dynamics_eq_pt(t, x, params), t, x0);

% extract the states
z = x(:, 1);
x = x(:, 2:3);

% compute the manifold, M = {x | psi(x) - Kappa(pi(x)) = 0}
% x_2 - (-k * z) = 0   <=>  x_2 + k * x1 = 0  <=>  x_2 = -k * x_1
x1_min = min(x(:,1));
x1_max = max(x(:,1));

x1_range = linspace(x1_min, x1_max, 100);
x2_range = -params.k_single * x1_range;
M = [x1_range; x2_range];

% plot the results
figure;

% plot the tracking
subplot(1,2,1);
hold on; grid on;
yline(0, 'k--');

%  plot the solutions
plot(t, z, 'b', 'LineWidth', 1.5);      % single integrator
plot(t, x(:,1), 'r', 'LineWidth', 1.5); % double integrator
legend('', 'Single', 'Double');

% plot double integrator traj
subplot(1,2,2);
hold on; grid on; axis equal;
xline(0); yline(0);
plot(x(:,1), x(:,2), 'b', 'LineWidth', 1.5);
plot(M(1,:), M(2,:), 'k--', 'LineWidth', 1.5);
legend('TraDoublejectory', 'Manifold');
xlabel('p');
ylabel('v');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% function that does both the double and single integrator
function xdot = dynamics_eq_pt(t, x, params)

    % unpack the state
    p_single = x(1);
    z = p_single;

    p_double = x(2);
    v_double = x(3);
    x = [p_double;
         v_double];

    % compute the control input for the single integrator
    v = -params.k_single * (z);  % v = Kappa(pi(x))

    % single integrator dynamics
    zdot = v;
    
    % compute the control input for the double integrator
    u = -params.kd_double * (x(2) - v);

    % double integrator dynamics
    x = [p_double;
         v_double];
    A_double = [0, 1;
                0, 0];
    B_double = [0;
                1];
    xdot = A_double * x + B_double * u;

    xdot = [zdot; xdot];
end

