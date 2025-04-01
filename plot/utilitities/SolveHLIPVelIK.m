function [Q_1_dot, Q_2_dot, L_dot] = SolveHLIPVelIK(P, V, Q_1, Q_2, m, g, l_thigh, l_shin)

N = length(P);

Q_1_dot = zeros(N, 1);
Q_2_dot = zeros(N, 1);
L_dot = zeros(N, 1);

for i = 1:N
    L_dot(i) = m * g * P(i);

    v_x = V(i);
    v_z = 0.0;
    v = [v_x; v_z];
    J = GetStfToBaseJacobian(Q_1(i), Q_2(i), l_thigh, l_shin);
    q_vel = inv(J) * v;
    Q_1_dot(i) = q_vel(1);
    Q_2_dot(i) = q_vel(2);
end

end
