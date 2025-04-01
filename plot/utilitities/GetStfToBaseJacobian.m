function J = GetStfToBaseJacobian(q_1, q_2, l_thigh, l_shin)

s1 = sin(q_1);
s12 = sin(q_1 + q_2);

c1 = cos(q_1);
c12 = cos(q_1 + q_2);

J = zeros(2, 2);

J(1, 1) = l_shin * c1 + l_thigh * c12;
J(2, 1) = -l_shin * s1 - l_thigh * s12;

J(1, 2) = l_thigh * c12;
J(2, 2) = -l_thigh * s12;

end