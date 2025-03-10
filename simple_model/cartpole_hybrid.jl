using LinearAlgebra
using DifferentialEquations
using ForwardDiff
using Plots

# Dynamics of Cartpole
m = 0.5
M = 1.0
l = 1.0
g = 9.8
γ = 0.00
D(q) = [m+M   m*l*cos(q[2]); m*l*cos(q[2]) m*l^2]
H(q, q̇) = [m * l * q̇[2]^2 * sin(q[2]), m * g * l * sin(q[2])]
B = [1.0, 0.0]
q̈(q, q̇, u) = inv(D(q)) * (B * u - H(q, q̇))

# Define walls
xmax = 1.0
xmin = -1.0

# Function for simulation
function odefun(dx, x, p, t)
	# Pull out states
	q = x[1:2]
	q̇ = x[3:4]

	# Get input
	u = 0.0

	# Assign derivative
	dx[1:2] .= q̇
	dx[3:4] .= q̈(q, q̇, u)
end

# Callbacks for hitting the walls
condition1(x, t, integrator) = x[1] - xmax
condition2(x, t, integrator) = x[1] - xmin
function affect1!(integrator)
	integrator.u[3] = -integrator.u[3]
end
function affect2!(integrator)
	integrator.u[3] = -integrator.u[3]
end
cb1 = ContinuousCallback(condition1, affect1!)
cb2 = ContinuousCallback(condition2, affect2!)
cb = CallbackSet(cb1, cb2)

# Sim parameters
x0 = [0.0, 0.00, 1.0, 0.0]
T = 20.0

# Run sim
sol = solve(ODEProblem(odefun, x0, (0.0, T)), callback = cb)

# Plot position
begin
	fig1 = plot(sol, idxs = 1, xlabel = raw"$t$", ylabel = raw"$x(t)$", label = "")
	hline!([xmin, xmax], c = :black)
end

# Plot velocity
begin
	fig2 = plot(sol, idxs = 3, xlabel = raw"$t$", ylabel = raw"$\dot{x}(t)$", label = "")
end

# Plot position vs velocity
begin
	fig3 = plot(sol, idxs = (1, 3), xlabel = raw"$x$", ylabel = raw"$\dot{x}$", label = "")
end

# Plot pendulum position
begin
	fig4 = plot(sol, idxs = 2, xlabel = raw"$t$", ylabel = raw"$\theta(t)$", label = "")
end

# Plot pendulum velocity
begin
	fig4 = plot(sol, idxs = 4, xlabel = raw"$t$", ylabel = raw"$\dot{\theta}(t)$", label = "")
end