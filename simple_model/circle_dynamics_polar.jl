# Load Packages
using LinearAlgebra
using DifferentialEquations
using ForwardDiff
using Plots
using NonlinearSolve

# Plot settings
default(grid = false, framestyle = :box, fontfamily = "Computer Modern", lw = 2.0, palette = :tab10, label = "")

# Step parameters
step_height = 0.1
step_length = 0.5

# Get circle
function circle_constraints((R, y, θ), (step_length, step_height))
	[
		y + R - step_height,
		y + R * sin(θ),
		R * cos(θ) - step_length / 2,
	]
end
circle_params = solve(NonlinearProblem(circle_constraints, [2 * step_length, -0.5, π / 4], [step_length, step_height]))
R = circle_params.u[1]
yc = circle_params.u[2]
θ = circle_params.u[3]
xc = 0.0 # Center always at zero
h(x) = (x[1] - xc)^2 + (x[2] - yc)^2 - R^2

# Dynamics in polar coordinates
K = 10.0
τ = 5.0
f(r, θ, K, τ) = [-K * (r - R), -τ]

# Recover cartesian
x(r, θ) = r * cos(θ)
y(r, θ) = r * sin(θ)

# Initial conditions
r0 = R - 0.1
θ0 = π + θ

# ODE function
function odefun(X, p, t)
	r, θ = X
	K, τ = p

	return f(r, θ, K, τ)
end

# Solve!
T = 10.0
sol = solve(ODEProblem(odefun, [r0, -θ0], (0.0, T), [K, τ]))

# Plot solution
begin
	plot(sol, xlabel = raw"$t$")
end

# Convert to cartisian
ts = 0.0:0.01:T
begin
	plot(xc .+ x.(sol.(ts, idxs = 1), sol.(ts, idxs = 2)), yc .+ y.(sol.(ts, idxs = 1), sol.(ts, idxs = 2)))
end
