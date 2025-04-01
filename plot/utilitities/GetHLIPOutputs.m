function [p, v, q_2] = GetHLIPOutputs(q_1, L, l_thigh, l_shin, m, z)

q_2 = - acos((z - l_shin * cos(q_1)) / (l_thigh)) - q_1;

% if(q_2 > 0)
%     q_2 = -q_2;
% end

p = l_shin * sin(q_1) + l_thigh * sin(q_1 + q_2);

v = L / (m * z);

% debug
z_test = l_shin * cos(q_1) + l_thigh * cos(q_1 + q_2);

end