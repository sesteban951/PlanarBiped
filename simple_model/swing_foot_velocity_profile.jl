# Load Packages
using LinearAlgebra
using DifferentialEquations
using ForwardDiff
using Plots

# Plot settings
default(grid=false, framestyle=:box, fontfamily="Computer Modern", lw=2.0, palette=:tab10, label="")

# Step parameters
step_height = 0.2
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
h(x) = smooth_min(h1(x), h2(x))

# Take a look at ellipse
begin
    xs = -1.0:0.01:1.0
    zs = -0.5:0.01:0.5
    fig1 = plot(xlabel=raw"$x$", ylabel=raw"$z$", xlims=(-1.0, 1.0), ylims=(-0.1, 0.4))
    contour!(xs, zs, (x, y) -> h1([x, y]), levels=[0.0], lw=2, c=1, colorbar=false)
    contour!(xs, zs, (x, y) -> h2([x, y]), levels=[0.0], lw=2, c=1, colorbar=false)
    hspan!([-0.5, 0.0], c=:gray, alpha=1.0)
end

# Dynamics forcontrol design
f(x) = zeros(2)
g(x) = diagm(ones(2))

# Desired vector field
kd(x) = [0.5, -0.5 * x[1]]

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
x0 = [-0.5, 0.00]
T = 10.0
sol = solve(ODEProblem((x, p, t) -> f(x) + g(x) * k(x), x0, (0.0, T)))

# Plot solution
begin
    xs = -1.0:0.01:1.0
    zs = -0.5:0.01:0.5
    fig2 = plot(xlabel=raw"$x$", ylabel=raw"$z$", xlims=(-1.0, 1.0), ylims=(-0.1, 0.4))
    contour!(xs, zs, (x, y) -> h1([x, y]), levels=[0.0], lw=2, c=4, colorbar=false)
    contour!(xs, zs, (x, y) -> h2([x, y]), levels=[0.0], lw=2, c=4, colorbar=false)
    hspan!([-0.5, 0.0], c=:gray, alpha=1.0)
    plot!(sol, idxs=(1, 2), c=1, label="", xlabel=raw"$x$", ylabel=raw"$z$")
end

# Now add a reset map so that we keep stepping
condition(x, t, integrator) = x[2] + 0.001
function affect!(integrator)
    integrator.u[1] = -0.5
    integrator.u[2] = 0.0
end
cb = ContinuousCallback(condition, affect!)

# Run a new sim
T2 = 20.0
sol2 = solve(ODEProblem((x, p, t) -> f(x) + g(x) * k(x), x0, (0.0, T2)), callback=cb)

# Plot solution
begin
    xs = -1.0:0.01:1.0
    zs = -0.5:0.01:0.5
    fig3 = plot(xlabel=raw"$x$", ylabel=raw"$z$", xlims=(-1.0, 1.0), ylims=(-0.1, 0.4))
    contour!(xs, zs, (x, y) -> h1([x, y]), levels=[0.0], lw=2, c=4, colorbar=false)
    contour!(xs, zs, (x, y) -> h2([x, y]), levels=[0.0], lw=2, c=4, colorbar=false)
    hspan!([-0.5, 0.0], c=:gray, alpha=1.0)
    plot!(sol2, idxs=(1, 2), c=1, label="", xlabel=raw"$x$", ylabel=raw"$z$")
end

# Make a gif of this
dt = 0.05
anim = @animate for t in 0.0:dt:T2
    plot(xlabel=raw"$x$", ylabel=raw"$z$", xlims=(-1.0, 1.0), ylims=(-0.1, 0.4))
    contour!(xs, zs, (x, y) -> h1([x, y]), levels=[0.0], lw=2, c=4, colorbar=false)
    contour!(xs, zs, (x, y) -> h2([x, y]), levels=[0.0], lw=2, c=4, colorbar=false)
    hspan!([-0.5, 0.0], c=:gray, alpha=1.0)
    scatter!([sol2(t, idxs=1)], [sol2(t, idxs=2)], ms=6, c=1)
    if t < 0.2
        plot!(sol2.(0.0:dt:t, idxs=1), sol2.(0.0:dt:t, idxs=2), c=1)
    else
        plot!(sol2.(t-0.2:dt:t, idxs=1), sol2.(t-0.2:dt:t, idxs=2), c=1)
    end
end
gif(anim, fps=15)