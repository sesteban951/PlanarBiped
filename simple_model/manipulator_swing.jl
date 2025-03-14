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
R = 0.1 #circle_params.u[1]
yc = -0.9 #circle_params.u[2]
θ = circle_params.u[3]
xc = 0.9 # Center always at zero
h(x) = (x[1] - xc)^2 + (x[2] - yc)^2 - R^2

# Circle dynamics
γ = 5.0
f(x) = [(x[2] - yc) + γ * (x[1] - xc) * (R^2 - (x[1] - xc)^2 - (x[2] - yc)^2), -(x[1] - xc) + γ * (x[2] - yc) * (R^2 - (x[1] - xc)^2 - (x[2] - yc)^2)]


# Manipulator parameters
m1 = 1.0;        # mass of link 1
m2 = 1.0;        # mass of link 2
l1 = 1.0;        # length of link 1
l2 = 1.0;        # length of link 2
b1 = 0.05;       # damping
b2 = 0.05;       # damping
g = 9.81;        # gravity

# compute the inertias of the links
l1_c = l1 / 2;
l2_c = l2 / 2;
# I1 = m1 * l1_c^2; # point mass at center of the link
# I2 = m2 * l2_c^2; # point mass at center of the link
I1 = (1 / 3) * m1 * l1^2; # thin rod
I2 = (1 / 3) * m2 * l2^2; # thin rod

# Inertia matrix
function D((q1, q2))
	D11 = I1 + I2 + m2 * l1^2 + 2 * m2 * l1 * l2_c * cos(q2)
	D12 = I2 + m2 * l1 * l2_c * cos(q2)
	D21 = D12
	D22 = I2

	return [D11 D12; D21 D22]
end

# Coriolis matrix
function C((q1, q2), (q1dot, q2dot))
	C11 = -2 * m2 * l1 * l2_c * sin(q2) * q2dot
	C12 = -m2 * l1 * l2_c * sin(q2) * q2dot
	C21 = m2 * l1 * l2_c * sin(q2) * q1dot
	C22 = 0
	C_dyn = [C11 C12; C21 C22]
	C_damping = [b1 0; 0 b2]

	return C_dyn + C_damping
end

# Gravity vector
function G((q1, q2))
	G1 = -m1 * g * l1_c * sin(q1) - m2 * g * (l1 * sin(q1) + l2_c * sin(q1 + q2))
	G2 = -m2 * g * l2_c * sin(q1 + q2)
	return -[G1, G2]
end

# Actuation matrix
B = [1.0 0.0; 0.0 1.0]

# Acceleration
q̈(q, q̇, u) = D(q) \ (B * u - C(q, q̇) * q̇)

# compute the jacobian
J((q1, q2)) = [
	l1*cos(q1)+l2*cos(q1 + q2)       l2*cos(q1 + q2);
	l1*sin(q1)+l2*sin(q1 + q2)       l2*sin(q1 + q2)
];

# forward kinematics of the manipulator
function forward_kin(q, qdot)

	# compute the elbow position
	p_elbow = [l1 * sin(q[1]);
		-l1 * cos(q[1])]

	# compute the hand position
	p_hand = [l1 * sin(q[1]) + l2 * sin(q[1] + q[2]);
		-l1 * cos(q[1]) - l2 * cos(q[1] + q[2])]

	# compute the hand velocity
	pdot_hand = J(q) * qdot

	return [p_elbow, p_hand, pdot_hand]
end

# inverse kinematics of the manipulator
function inverse_kin(p_hand_des, pdot_hand_des)

	# compute the inverse kinematics
	x = p_hand_des[1]
	z = p_hand_des[2]
	L = sqrt(x^2 + z^2)

	# elbow angle (be aware that there are two solutions, elbow up and elbow down)
	gamma = acos((L^2 - l1^2 - l2^2) / (-2 * l1 * l2))
	q2 = gamma - pi

	# shoulder angle
	beta = atan(x, -z) # Keep an EYE
	alpha = acos((l2^2 - l1^2 - L^2) / (-2 * l1 * L))
	q1 = beta + alpha

	# IK solution 
	q_sol = [q1; q2]

	# compute the joint velocities via inverse Jacobian
	qdot_sol = J(q_sol) \ pdot_hand_des

	return [q_sol, qdot_sol]
end

# Get my desired q̇
function q̇dJ(x)
	q, q̇ = x[1:2], x[3:4]
	p_hand = forward_kin(q, q̇)[2]
	ṗd_hand = f(p_hand)
	q̇d = inverse_kin(p_hand, ṗd_hand)[2]
end

dq̇dJ(x) = ForwardDiff.jacobian(q̇dJ, x)

ForwardDiff.jacobian(x -> J(x[1:2]) * x[3:4], x0)

function odefun(dx, x, p, t)
	# Pull out states
	q = x[1:2]
	q̇ = x[3:4]

	# Get hand position
	p_elbow, p_hand, ṗ_hand = forward_kin(q, q̇)

	# Desired p_hand
	# pd_hand = [1.0, -1.0]

	# Desired hand velocity
	ṗd_hand = f(p_hand)

	# Get desired configuration
	qd, q̇d = inverse_kin(p_hand, ṗd_hand)

	# Control input
	Kp = 50.0
	Kd = 25.0
	u = -Kp * (q - qd) - Kd * (q̇ - q̇d) + C(q, q̇) * q̇

	# Now assign dynamics
	dx[1:2] .= q̇
	dx[3:4] .= q̈(q, q̇, u)
end

# Plot circle to get an idea of where we are
begin
	xs = xc-R-0.1:0.01:xc+R+0.1
	ys = yc-R-0.1:0.01:yc+R+0.1
	plot(xlabel = raw"$x$", ylabel = raw"$z$", xlims = (xc - R - 0.1, xc + R + 0.1), ylims = (yc - R - 0.1, yc + R + 0.1), aspect_ratio = :equal)
	contour!(xs, ys, (x, y) -> h([x, y]), c = :black, levels = [0.0], colorbar = false)
end

# Now we will run our
q0 = [0.3, 0.3]
q̇0 = [0.0, 0.0]
x0 = [q0; q̇0]
T = 100.0
sol = solve(ODEProblem(odefun, x0, (0.0, T)))

# Look at desired dynamics
p0 = [xc, yc + 0.001]
sol2 = solve(ODEProblem((x, p, t) -> f(x), p0, (0.0, T)))


# Unpack solution
q(t) = sol(t, idxs = 1:2)
q̇(t) = sol(t, idxs = 3:4)
p_hand(t) = forward_kin(q(t), q̇(t))[2]

# Plot 
ts = 0.0:0.05:T
begin
	xs = xc-R-0.1:0.01:xc+R+0.1
	ys = yc-R-0.1:0.01:yc+R+0.1
	plot(xlabel = raw"$x$", ylabel = raw"$z$", aspect_ratio = :equal)
	contour!(xs, ys, (x, y) -> h([x, y]), c = :black, levels = [0.0], colorbar = false)
	plot!([p_hand(t)[1] for t in ts], [p_hand(t)[2] for t in ts], c = 1)
	plot!(sol2, idxs = (1, 2), c = 2, label = "")
end

# plot([p_hand(t)[1] for t in ts], [p_hand(t)[2] for t in ts], c = 1)
# plot(sol, idxs = 3:4)

# plot(ts, rank.(J.(sol.(ts, idxs = 1:2))))
