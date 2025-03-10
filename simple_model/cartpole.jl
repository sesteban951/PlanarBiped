using LinearAlgebra
using DifferentialEquations
using ForwardDiff
using Plots

# Dynamics of Cartpole
m = 0.5
M = 1.0
l = 1.0
g = 9.8
D(q) = [m+M   m*l*cos(q[2]); m*l*cos(q[2]) m*l^2]
H(q, q̇) = [m * l * q̇[2]^2 * sin(q[2]), m * g * l * sin(q[2])]
B = [1.0, 0.0]

# Desired trajectory
xd(t) = sin(t)
ẋd(t) = ForwardDiff.derivative(xd, t)
ẍd(t) = ForwardDiff.derivative(ẋd, t)

# RoM input
Kp = 5.0
κ(x, t) = -Kp * (x - xd(t))
∂κ∂x(x, t) = ForwardDiff.derivative(x -> κ(x, t), x)
∂κ∂t(x, t) = ForwardDiff.derivative(t -> κ(x, t), t)
dκ(x, ẋ, t) = ∂κ∂x(x, t) * ẋ + ∂κ∂t(x, t)

# Track reduced-order model input
Kd = 10.0
k(q, q̇, t) = -Kd * (q̇[1] - κ(q[1], t)) + dκ(q[1], q̇[1], t)

# Acceleration
q̈(q, q̇, u) = D(q) \ (-H(q, q̇) + B * u)

# Function for simulation
function odefun(dx, x, p, t)
	# Pull out states
	q = x[1:2]
	q̇ = x[3:4]

	# Get input
	u = k(q, q̇, t)

	# Assign derivative
	dx[1:2] .= q̇
	dx[3:4] .= q̈(q, q̇, u)
end

# Simulate
T = 100.0
ts = 0.0:0.05:T
x0 = zeros(4)
sol = solve(ODEProblem(odefun, x0, (0.0, T)))

θ(t) = sol(t, idxs = 2)
dθ(t) = sol(t, idxs = 4)
ts2 = 50.0:0.05:T

# Plot stuff
default(grid = false, framestyle = :box, fontfamily = "Computer Modern", lw = 2.0, guidefont = 12, tickfont = 0, legendfont = 10, palette = :tab10, label = "")

# Plot position
begin
	fig1 = plot(sol, idxs = 1:2, xlabel = raw"$t$", ylabel = raw"Position", label = [raw"$x(t)$" raw"$\theta(t)$"])
	plot!(ts, xd.(ts), c = 3, ls = :dot, label = raw"$x_{d}(t)$")
end

# Plot position vs velocity
begin
	fig2 = plot(sol, idxs = (1, 3), xlabel = raw"$x$", ylabel = raw"$\dot{x}$", label = "")
end

begin
	fig3 = plot(sol, idxs = (2, 4), xlabel = raw"$\theta$", ylabel = raw"$\dot{\theta}$", label = "")
end