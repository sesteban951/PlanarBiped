function [Z, Z_dot] = ConvertHLIPToZeroDynamics(P, V, z, l_thigh, l_shin, m)

N = length(P);

Z = zeros(N, 1);
Z_dot = zeros(N, 1);

for i = 1:N
    Z(i) = SolveHLIPIK(P(i), z, l_thigh, l_shin);
    Z_dot(i) = V(i) * z * m;
end