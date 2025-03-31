function q_1 = SolveHLIPIK(p, z, l_thigh, l_shin)

L_stf = sqrt(p * p + z * z);

beta_stf = acos((l_thigh * l_thigh + l_shin * l_shin - L_stf * L_stf) / (2.0 * l_thigh * l_shin));

q_stf_knee = beta_stf - pi;

mu_stf = atan2(p, z);

gamma_stf = asin((l_shin / L_stf) * sin(beta_stf)); 

q_stf_ankle = mu_stf + gamma_stf;

q_1 = q_stf_ankle;

end