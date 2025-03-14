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
tube_radius = 0.05

# Construct ellipse for swing foot trajectory
P1 = [1.0/(step_length-tube_radius)^2 0.0; 0.0 1.0/(step_height-tube_radius)^2]
P2 = [1.0/(step_length+tube_radius)^2 0.0; 0.0 1.0/(step_height+tube_radius)^2]
h1(x) = x' * P1 * x - 1.0
h2(x) = 1.0 - x' * P2 * x

# Smoothly combine both CBFs
β = 0.1
smooth_min(x, y) = -β * log(exp(-x / β) + exp(-y / β))
# h(x) = smooth_min(h1(x), h2(x))

# Take a look at ellipse
begin
	xs = -1.0:0.01:1.0
	zs = -0.5:0.01:0.5
	fig1 = plot(xlabel = raw"$x$", ylabel = raw"$z$", xlims = (-1.0, 1.0), ylims = (-0.1, 0.4))
	contour!(xs, zs, (x, y) -> h1([x, y]), levels = [0.0], lw = 2, c = 1, colorbar = false)
	contour!(xs, zs, (x, y) -> h2([x, y]), levels = [0.0], lw = 2, c = 1, colorbar = false)
	hspan!([-0.5, 0.0], c = :gray, alpha = 1.0)
end

# Actually may be better to find two circles
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
xc = 0.0 # Center always at zero
h3(x) = (x[1] - xc)^2 + (x[2] - yc)^2 - R^2

# Now create tube around step
h4(x) = (x[1] - xc)^2 + (x[2] - yc)^2 - (R - tube_radius)^2
h5(x) = (x[1] - xc)^2 + (x[2] - yc)^2 - (R + tube_radius)^2

# Look at circle
begin
	xs = -1.0:0.01:1.0
	zs = -0.5:0.01:0.5
	plot(xlabel = raw"$x$", ylabel = raw"$z$", xlims = (-0.5, 0.5), ylims = (-0.1, 0.4))
	contour!(xs, zs, (x, y) -> h3([x, y]), levels = [0.0], lw = 2, c = 4, alpha = 0.7, colorbar = false)
	contour!(xs, zs, (x, y) -> h4([x, y]), levels = [0.0], lw = 2, c = 1, colorbar = false)
	contour!(xs, zs, (x, y) -> h5([x, y]), levels = [0.0], lw = 2, c = 1, colorbar = false)
	hspan!([-0.5, 0.0], c = :gray, alpha = 1.0)
end

# Combine barriers
h(x) = min(h4(x), -h5(x))

# Dynamics forcontrol design
f(x) = zeros(2)
g(x) = diagm(ones(2))

# Desired vector field
kd(x) = [0.5, -x[1]]

# Barrier params
α = 1.0
∇h(x) = ForwardDiff.gradient(h, x)
Lfh(x) = ∇h(x)'f(x)
Lgh(x) = ∇h(x)'g(x)
a(x) = Lfh(x) + Lgh(x) * kd(x) + α * h(x)
b(x) = norm(Lgh(x))^2
λ(a, b) = b == 0.0 ? 0.0 : max(0.0, -a / b)
k(x) = kd(x) + λ(a(x), b(x)) * Lgh(x)'

# Solve ODE
x0 = [-0.25, 0.00]
T = 10.0
sol = solve(ODEProblem((x, p, t) -> f(x) + g(x) * k(x), x0, (0.0, T)))

# Plot solution
begin
	xs = -1.0:0.01:1.0
	zs = -0.5:0.01:0.5
	fig2 = plot(xlabel = raw"$x$", ylabel = raw"$z$", xlims = (-0.5, 0.5), ylims = (-0.1, 0.4))
	contour!(xs, zs, (x, y) -> h4([x, y]), levels = [0.0], lw = 2, c = 4, colorbar = false)
	contour!(xs, zs, (x, y) -> h5([x, y]), levels = [0.0], lw = 2, c = 4, colorbar = false)
	hspan!([-0.5, 0.0], c = :gray, alpha = 1.0)
	plot!(sol, idxs = (1, 2), c = 1, label = "", xlabel = raw"$x$", ylabel = raw"$z$")
end

# Now add a reset map so that we keep stepping
condition(x, t, integrator) = x[2] + 0.00001
function affect!(integrator)
	integrator.u[1] = -0.25
	integrator.u[2] = 0.0
end
cb = ContinuousCallback(condition, affect!)

# Run a new sim
T2 = 20.0
sol2 = solve(ODEProblem((x, p, t) -> f(x) + g(x) * k(x), x0, (0.0, T2)), callback = cb)

# Plot solution
begin
	xs = -1.0:0.01:1.0
	zs = -0.5:0.01:0.5
	fig3 = plot(xlabel = raw"$x$", ylabel = raw"$z$", xlims = (-0.5, 0.5), ylims = (-0.1, 0.4))
	contour!(xs, zs, (x, y) -> h4([x, y]), levels = [0.0], lw = 2, c = 4, colorbar = false)
	contour!(xs, zs, (x, y) -> h5([x, y]), levels = [0.0], lw = 2, c = 4, colorbar = false)
	hspan!([-0.5, 0.0], c = :gray, alpha = 1.0)
	plot!(sol2, idxs = (1, 2), c = 1, label = "", xlabel = raw"$x$", ylabel = raw"$z$")
end

# Make a gif of this
dt = 0.05
anim = @animate for t in 0.0:dt:T2
	plot(xlabel = raw"$x$", ylabel = raw"$z$", xlims = (-0.5, 0.5), ylims = (-0.1, 0.4))
	contour!(xs, zs, (x, y) -> h4([x, y]), levels = [0.0], lw = 2, c = 4, colorbar = false)
	contour!(xs, zs, (x, y) -> h5([x, y]), levels = [0.0], lw = 2, c = 4, colorbar = false)
	hspan!([-0.5, 0.0], c = :gray, alpha = 1.0)
	scatter!([sol2(t, idxs = 1)], [sol2(t, idxs = 2)], ms = 6, c = 1)
	if t < 0.2
		plot!(sol2.(0.0:dt:t, idxs = 1), sol2.(0.0:dt:t, idxs = 2), c = 1)
	else
		plot!(sol2.(t-0.2:dt:t, idxs = 1), sol2.(t-0.2:dt:t, idxs = 2), c = 1)
	end
end
gif(anim, fps = 15)


# CLF for staying on swing foot trajectory
y(x) = h3(x)
V(x) = 0.5 * y(x)^2
∇V(x) = ForwardDiff.gradient(V, x)
LfV(x) = ∇V(x)' * f(x)
LgV(x) = ∇V(x)' * g(x)
γ = 0.2

# Desired vector field: move to the right
kd(x) = [0.5, 0.0]

# "Filter" This desired vector field with the CLF to keep us on the nominal path
λV(a, b) = b == 0.0 ? 0.0 : max(0.0, a / b)
kV(x) = kd(x) - λV(LfV(x) + LgV(x) * kd(x) + γ * V(x), norm(LgV(x))^2) * LgV(x)'

# Run sim under this controller
sol3 = solve(ODEProblem((x, p, t) -> f(x) + g(x) * kV(x), x0, (0.0, 3 * T)))

# Plot closed-loop vector field as well
vector_scale = 0.025
xv = -0.5:0.025:0.5
yv = 0.0:0.025:0.5
Xv = [x for x in xv for y in yv]
Yv = [y for x in xv for y in yv]
fcl(x) = f(x) + g(x) * kV(x)
Xf = [vector_scale * normalize(fcl([x, y]))[1] for (x, y) in zip(Xv, Yv)]
Yf = [vector_scale * normalize(fcl([x, y]))[2] for (x, y) in zip(Xv, Yv)]

# Plot solution
begin
	xs = -1.0:0.01:1.0
	zs = -0.5:0.01:0.5
	fig2 = plot(xlabel = raw"$x$", ylabel = raw"$z$", xlims = (-0.5, 0.5), ylims = (-0.1, 0.4))
	contour!(xs, zs, (x, y) -> h3([x, y]), levels = [0.0], lw = 2, c = 4, alpha = 0.5, colorbar = false)
	hspan!([-0.5, 0.0], c = :gray, alpha = 1.0)
	plot!(sol3, idxs = (1, 2), c = 1, label = "", xlabel = raw"$x$", ylabel = raw"$z$")
	# quiver!(Xv, Yv, quiver = (Xf, Yf), c = 5, lw = 1.5)
end

# Now run the same sim but we reset
T2 = 20.0
sol4 = solve(ODEProblem((x, p, t) -> f(x) + g(x) * kV(x), x0, (0.0, T2)), callback = cb)

# Make a gif of this
dt = 0.025
anim = @animate for t in 0.0:dt:T2
	plot(xlabel = raw"$x$", ylabel = raw"$z$", xlims = (-0.5, 0.5), ylims = (-0.1, 0.4))
	contour!(xs, zs, (x, y) -> h3([x, y]), levels = [0.0], lw = 2, c = 4, colorbar = false)
	hspan!([-0.5, 0.0], c = :gray, alpha = 1.0)
	scatter!([sol4(t, idxs = 1)], [sol4(t, idxs = 2)], ms = 6, c = 1)
	if t < 0.05
		plot!(sol4.(0.0:dt:t, idxs = 1), sol4.(0.0:dt:t, idxs = 2), c = 1, alpha = 0.3)
	else
		plot!(sol4.(t-0.2:dt:t, idxs = 1), sol4.(t-0.2:dt:t, idxs = 2), c = 1, alpha = 0.4)
	end
end
gif(anim, fps = 15)
# gif(anim, "simple_model/swing_foot_velocity_profile.gif", fps = 15)

# What if we use two barriers to constrain us to the surface?
h(x) = min(h3(x), -h3(x))

# Desired vector field
vx = 0.5 # Desired speed to move forward
kd(x) = [vx, 0.0]

# Barrier params
α = 1.0
∇h(x) = ForwardDiff.gradient(h, x)
Lfh(x) = ∇h(x)'f(x)
Lgh(x) = ∇h(x)'g(x)
a(x) = Lfh(x) + Lgh(x) * kd(x) + α * h(x)
b(x) = norm(Lgh(x))^2
λ(a, b) = b == 0.0 ? 0.0 : max(0.0, -a / b)
k(x) = kd(x) + λ(a(x), b(x)) * Lgh(x)'

# Solve ODE
x0 = [-0.25, 0.00]
T = 10.0
sol5 = solve(ODEProblem((x, p, t) -> f(x) + g(x) * k(x), x0, (0.0, T)))

# Plot closed-loop vector field as well
vector_scale = 0.025
xv = -0.5:0.025:0.5
yv = 0.0:0.025:0.5
Xv = [x for x in xv for y in yv]
Yv = [y for x in xv for y in yv]
fcl(x) = f(x) + g(x) * k(x)
Xf = [vector_scale * normalize(fcl([x, y]))[1] for (x, y) in zip(Xv, Yv)]
Yf = [vector_scale * normalize(fcl([x, y]))[2] for (x, y) in zip(Xv, Yv)]

# Plot solution
begin
	xs = -1.0:0.01:1.0
	zs = -0.5:0.01:0.5
	fig2 = plot(xlabel = raw"$x$", ylabel = raw"$z$", xlims = (-0.5, 0.5), ylims = (-0.1, 0.4))
	contour!(xs, zs, (x, y) -> h3([x, y]), levels = [0.0], lw = 2, c = 4, alpha = 0.5, colorbar = false)
	hspan!([-0.5, 0.0], c = :gray, alpha = 1.0)
	plot!(sol5, idxs = (1, 2), c = 1, label = "", xlabel = raw"$x$", ylabel = raw"$z$")
	quiver!(Xv, Yv, quiver = (Xf, Yf), c = 5, lw = 1.5)
end