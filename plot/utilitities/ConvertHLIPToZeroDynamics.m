function [Q_1, Q_2, L] = ConvertHLIPToZeroDynamics(P, V, z, l_thigh, l_shin, m)

N = length(P);

Q_1 = zeros(N, 1);
Q_2 = zeros(N, 1);
L = zeros(N, 1);

for i = 1:N
    [Q_1(i), Q_2(i)] = SolveHLIPIK(P(i), z, l_thigh, l_shin);
    L(i) = V(i) * z * m;
end