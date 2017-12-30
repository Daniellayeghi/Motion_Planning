% function that resolves for all joint space trajectories besides the
% maximum time one 

% function [pos, vel, T_j_1, T_j_2, T_a, T_d, T_v, v_lim,T] = optimize_joint(pos, vel, T_j_1, T_j_2, T_a, T_d, T_v, a_lim_a, j_max, v_lim,T)

% [max_T,joint] = max(T);
% [row_max, ~] = ind2sub(size(T), joint);

for i = 1:6 
    if i ~= row_max 
        a_lim_a(i,1) = acc_max(i,1);
        a_lim_d(i,1) = acc_min(i,1); 
        v_lim(i,1) = (-a_lim_a(i,1)^2+a_lim_a(i,1)*j_max(i,1)*max_T - sqrt(a_lim_a(i,1)*(-4*(pos(i,2) - pos(i,1))*j_max(i,1)^2+a_lim_a(i,1)*((a_lim_a(i,1)-j_max(i,1)*max_T)^2))))/(2*j_max(i,1));
        T_a(i,1) = (a_lim_a(i,1)^2+a_lim_a(i,1)*j_max(i,1)*max_T - sqrt(a_lim_a(i,1)*(-4*(pos(i,2) - pos(i,1))*j_max(i,1)^2+a_lim_a(i,1)*((a_lim_a(i,1)-j_max(i,1)*max_T)^2))))/(2*a_lim_a(i,1)*j_max(i,1));
        T_d(i,1) = T_a(i,1);
        T_j_1(i,1) = a_lim_a(i,1)/j_max(i,1);
        T_j_2(i,1) = T_j_1(i,1);
        T(i,1) = max_T;
        T_v(i,1) = T(i,1)-(T_a(i,1) + T_d(i,1)); 
    end 
%     return
end 





