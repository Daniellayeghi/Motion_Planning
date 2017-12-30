        



a_lim_a=acc_max ;
a_lim_d=acc_min; 
v_lim= (-a_lim_a^2+a_lim_a*j_max*max_T - sqrt(a_lim_a*(-4*(pos(2,1) - pos(1,1))*j_max^2+a_lim_a*((a_lim_a-j_max*max_T)^2))))/(2*j_max);

T_a = (a_lim_a^2+a_lim_a*j_max*max_T - sqrt(a_lim_a*(-4*(pos(2,1) - pos(1,1))*j_max^2+a_lim_a*((a_lim_a-j_max*max_T)^2))))/(2*a_lim_a*j_max);

T_d = T_a;
T_j_1 = a_lim_a/j_max;
T_j_2 = T_j_1;
T_1 = max_T;
T_v = T-(T_a + T_d); 