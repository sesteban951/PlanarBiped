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

# Unit circle dynamics
γ = 1.0
f(x) = [(x[2] - yc) + γ * x[1] * (R^2 - x[1]^2 - (x[2] - yc)^2), -(x[1] - xc) + γ * x[2] * (R^2 - x[1]^2 - (x[2] - yc)^2)]

# Solve from some initial codnition
x0 = [-R * cos(θ), 0.0]
T = 15.0
sol = solve(ODEProblem((x, p, t) -> f(x), x0, (0.0, T)))

# Plot
begin
	xs = xc-R-0.1:0.01:xc+R+0.1
	ys = yc-R-0.1:0.01:yc+R+0.1
	plot(xlabel = raw"$x$", ylabel = raw"$z$", xlims = (-0.5, 0.5), ylims = (-0.1, 0.5))
	contour!(xs, ys, (x, y) -> h([x, y]), c = :black, levels = [0.0], colorbar = false)
	plot!(sol, idxs = (1, 2), c = 1, label = "", axis_equal = true)
	hspan!([-0.5, 0.0], c = :gray, alpha = 1.0)
end

# Plot closed-loop vector field as well
vector_scale = 0.03
xv = -0.6:0.05:0.6
yv = -0.5:0.05:0.5
Xv = [x for x in xv for y in yv]
Yv = [y for x in xv for y in yv]
Xf = [vector_scale * normalize(f([x, y]))[1] for (x, y) in zip(Xv, Yv)]
Yf = [vector_scale * normalize(f([x, y]))[2] for (x, y) in zip(Xv, Yv)]

begin
	xs = xc-R-0.1:0.01:xc+R+0.1
	ys = yc-R-0.1:0.01:yc+R+0.1
	plot(xlabel = raw"$x$", ylabel = raw"$z$", xlims = (-0.5, 0.5), ylims = (-0.1, 0.3))
	contour!(xs, ys, (x, y) -> h([x, y]), c = :black, levels = [0.0], colorbar = false)
	plot!(sol, idxs = (1, 2), c = 1, label = "", axis_equal = true)
	quiver!(Xv, Yv, quiver = (Xf, Yf), c = 4, lw = 1.5)
	hspan!([-0.5, 0.0], c = :gray, alpha = 1.0)
end
