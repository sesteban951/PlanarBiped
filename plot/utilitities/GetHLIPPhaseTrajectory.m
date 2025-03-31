function [P, V] = GetHLIPPhaseTrajectory(z, g, v, T_SSP, T_DSP)

lambda = sqrt(g / z);

sigma_P1 = lambda * coth(0.5 * lambda * T_SSP);
p_minus_H = (v * (T_SSP + T_DSP)) / (2 + T_DSP * sigma_P1);
v_minus_H = sigma_P1 * (v * (T_SSP + T_DSP)) / (2 + T_DSP * sigma_P1);

x_f = [p_minus_H; v_minus_H];

A_SSP = [0, 1; lambda^2, 0];

res = 0.001;

T = 0:res:T_SSP;

N = length(T);

P = zeros(N, 1);
V = zeros(N, 1);

for i = 1:N
    t = -T(N - i + 1);
    x = expm(A_SSP * t) * x_f;
    P(i) = x(1);
    V(i) = x(2);
end

P(N + 1) = P(1);
V(N + 1) = V(1);

end