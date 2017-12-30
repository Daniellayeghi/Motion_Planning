function [x,v,a,j,time] = Disp_double_s (p_s, p_f, v_s, v_f, T_j_1, T_j_2, T_a, T_d, T_v, a_lim_a, a_lim_d, v_lim, j_max, j_min)

    T = T_a + T_v +T_d;
    
% assume sampling at 100Hz 
% in complete trajectory
    iters = round(T*100);

% arrays t hold position and it's derivatives
    x = zeros(iters,1);
    v = zeros(iters,1);
    a = zeros(iters,1);
    j = zeros(iters,1);
    time = zeros(iters,1);
    
% =========================================================================    
% assuming update rate of 0.01s
    for ts = 0:0.01:T
%         acceleration phase
        if ts <= T_j_1
             x(round(ts*100)+1) = p_s + v_s*ts +j_max*(ts^3/6);
             v(round(ts*100)+1) = v_s + j_max*(ts^2/2);
             a(round(ts*100)+1) = j_max *ts;
             j(round(ts*100)+1) = j_max;
        end 
        
        if ts > T_j_1 && ts <= T_a - T_j_1
             x(round(ts*100)+1) = p_s + v_s*ts + a_lim_a/6*(3*ts^2-3*T_j_1*ts+T_j_1^2);
             v(round(ts*100)+1) = v_s + a_lim_a*(ts-T_j_1/2);
             a(round(ts*100)+1) = j_max * T_j_1;
             j(round(ts*100)+1) = 0;
        end 
        
        if ts > T_a - T_j_1 && ts <= T_a
            x(round(ts*100)+1) = p_s +(v_lim+v_s)*T_a/2-v_lim*(T_a-ts) - j_min*(T_a-ts)^3/6;
            v(round(ts*100)+1) = v_lim + j_min*(T_a-ts)^2/2;
            a(round(ts*100)+1) = -j_min*(T_a - ts);
            j(round(ts*100)+1) = j_min;
            
        end 
%         constant velocity phase
        if ts > T_a && ts <= T_a + T_v 
           x(round(ts*100)+1) = p_s+(v_lim+v_s)*T_a/2+v_lim*(ts-T_a);
           v(round(ts*100)+1) = v_lim;
           a(round(ts*100)+1) = 0;
           j(round(ts*100)+1) = 0;
        end 
%         deceleration phase
        if ts > T - T_d && ts <= T - T_d + T_j_2
            x(round(ts*100)+1) = p_f - (v_lim + v_f)*T_d/2+v_lim*(ts-T+T_d) - j_max*(ts-T+T_d)^3/6;
            v(round(ts*100)+1) = v_lim - j_max*(ts-T+T_d)^2/2;
            a(round(ts*100)+1) = -j_max*(ts-T+T_d);
            j(round(ts*100)+1) = j_min;
        end 
        
        if ts > T - T_d + T_j_2 && ts <= T-T_j_2
            x(round(ts*100)+1) = p_f - (v_lim+v_f)*T_d/2+v_lim*(ts-T+T_d)+a_lim_d/6*(3*(ts-T+T_d)^2 - 3*T_j_2*(ts-T+T_d)+T_j_2^2);
            v(round(ts*100)+1) = v_lim+a_lim_d*(ts-T+T_d-T_j_2/2);
            a(round(ts*100)+1) = -j_max*T_j_2;
            j(round(ts*100)+1) = 0;
        end 
        
        if ts > T-T_j_2 
            x(round(ts*100)+1) = p_f - v_f*(T-ts) - j_max*(T-ts)^3/6;
            v(round(ts*100)+1) = v_f + j_max*(T-ts)^2/2;
            a(round(ts*100)+1) = -j_max*(T-ts);
            j(round(ts*100)+1) = j_max;
        end 
        
      
        time(round(ts*100)+1) = ts;

    end    
    return
end