%% display time dependent trajectory
% given some time spans this is a complete test file
% given parameters:
%  T_total;
%  T_acc;
%  T_j_1;
%  T_j_2;
%  pos;

%%
% T = round(T,2);
% T_a = round(T_a,2);
% T_d = round(T_d,2);
% T_j_1 = round(T_j_1,2);
% T_j_2 = round(T_j_2,2);
h = p_f - p_s ;
v_lim = h/(T-T_a);
a_lim_a = h/((T-T_a)*(T_a-T_j_1));
a_lim_d = a_lim_a;
j_max = h/((T - T_a)*(T_a - T_j_1)*T_j_1);
j_min = -j_max;
T_v = T - T_a - T_d;