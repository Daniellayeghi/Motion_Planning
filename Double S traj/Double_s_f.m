function[T_j_1, T_j_2, T_a, T_d, T_v, a_lim_a, a_lim_d, v_lim, j_max, j_min, p_s , p_f , v_f, v_s] = Double_s_f (pos_0, pos_1, vel_0, vel_1, vel_max, vel_min, acc_max, acc_min, jerk_max, jerk_min, dec_point)
%given the inputs for position start and final, velocity start and final,
%max and min constraints on acceleration

% calculate the new start and final position based on wether pos_1<pos_2
% and vel_1<vel_2;

rev = sign(pos_0-pos_1);

% new values for position 
% p_f = rev*pos_1;% 
% p_s = rev*pos_0; % 
p_s = pos_0; p_f = pos_1;

% velocity 
% v_s = rev*vel_0;
% v_f = rev*vel_1;
v_s = vel_0;v_f = vel_1;

%   new constraints for velocity and accelration based on rev
% v_max = ((rev+1)/2*vel_max + (rev-1)/2*vel_min);
% v_min = ((rev+1)/2*vel_min + (rev-1)/2*vel_max);
% a_max = ((rev+1)/2*acc_max + (rev-1)/2*acc_min);
% a_min = ((rev+1)/2*acc_min + (rev-1)/2*acc_max);
% j_max = ((rev+1)/2*jerk_max + (rev-1)/2*jerk_min);
% j_min = ((rev+1)/2*jerk_min + (rev-1)/2*jerk_max);

% v_max = vel_max;
% v_min = vel_min;
% a_max = acc_max;
% a_min = acc_min;
% j_max = jerk_max;
% j_min = jerk_min;
%=========================================================================%

%   calculate double jerk impulse time
    T_s_j = min(sqrt((v_f- v_s)/j_max), a_max/j_max);
%   check if trajectory is possible        
    %if (T_s_j < a_max && p_f - p_s > T_s_j*(v_f + v_s)) || (T_s_j == a_max && p_f - p_s > (1/2*(v_s + v_f)*(T_j_s + abs(v_f - v_s)/a_max)))
   
        % check if min and max accelrations are reached then calculate time
        % periods for velocity acceleration and jerk
        if (v_max - v_s)*j_max  < a_max^2
            disp("a_max not reached");
            T_j_1 = sqrt((v_max - v_s)/j_max);
            T_a = 2*T_j_1;
        else
            T_j_1 = a_max/j_max;
            T_a = T_j_1+(v_max-v_s)/a_max;
        end 

        if (v_max - v_f)*j_max  < a_max^2
            disp("a_min not reached");
            T_j_2 = sqrt((v_max - v_f)/j_max);
            T_d = 2*T_j_2;
        else 
            T_j_2 = a_max/j_max;
            T_d = T_j_2+(v_max-v_f)/a_max;
        end
        
%       calculate constant velocity time period;
        T_v = (p_f - p_s)/v_max - T_a/2*(1+v_s/v_max) - T_d/2*(1+v_f/v_max);
        
%       check if there is a constant velocit time period
        if T_v > 0
%           computation is done
            disp("maximum velocity reached");
            T_d = round(T_d, dec_point);
            T_a = round(T_a, dec_point);
            T_v = round(T_v, dec_point);
            T_j_1 = round(T_j_1, dec_point);
            T_j_2 = round(T_j_2, dec_point);
            v_lim = v_max;
            a_lim_a = a_max;
            a_lim_d = a_min;
        else
            disp("maximum velocity not reached");
            [T_j_1, T_j_2, T_a, T_d, T_v, a_lim_a, a_lim_d, v_lim, j_max, j_min] = v_lim_s_traj(p_f, p_s, v_f, v_s, a_max, a_min, j_max, j_min, dec_point);
        end

%   %  else 
%         disp("trajectory not possible");
%     end 
    return;
end 

