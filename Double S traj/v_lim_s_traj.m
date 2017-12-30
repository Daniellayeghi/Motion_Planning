function [T_j_1, T_j_2, T_a, T_d, T_v, a_lim_a, a_lim_d, v_lim, j_max, j_min] = v_lim_s_traj(p_f, p_s, v_f , v_s, a_max, a_min, j_max, j_min, dec_point)
    
    T_j = round(a_max/j_max, dec_point);
    T_j_1 = T_j;
    T_j_2 = T_j;
    T_v = 0;
    delta = (a_max^4/j_max^2)+2*(v_s^2+v_f^2)+a_max*(4*(p_f-p_s) - 2*a_max/j_max*(v_s+v_f));
    T_a = round(((a_max^2/j_max - 2*v_s+sqrt(delta))/(2*a_max)), dec_point);
    T_d = round(((a_max^2/j_max - 2*v_f+sqrt(delta))/(2*a_max)), dec_point);

    %solve for velocity and accelration limits

    if T_a < 0 && T_d > 0

        T_a = 0;
        T_d = round(2*(p_f - p_s)/(v_f - v_s), dec_point);
        T_j_2 = round((j_max*(p_f - p_s) - sqrt(j_max*(j_max*(p_f - p_s)^2+(v_f + v_s)^2*(v_f - v_s))))/(j_max*(v_f-v_s)), dec_point);

    elseif T_a > 0 && T_d < 0 

        T_d = 0; 
        T_a = round(2*(p_f - p_s)/(v_f - v_s), dec_point);
        T_j_1 = round((j_max*(p_f - p_s) - sqrt(j_max*(j_max*(p_f - p_s)^2+(v_f + v_s)^2*(v_f - v_s))))/(j_max*(v_f-v_s)), dec_point);
    end 
    a_lim_a = j_max *T_j_1;
    a_lim_d = -j_max *T_j_2;
    v_lim = v_s+(T_a-T_j_1)*a_lim_a;
    j_max;
    j_min;
    return;
end 