% create a double S trajectory for motio profile

function [T_j_1, T_j_2, T_a, T_d, T_v , v_lim, a_lim_a, a_lim_d, j_max, j_min, p_s, p_f, v_s, v_f] = Double_S(p_s, p_f, v_s, v_f, a_s, a_f, v_max_p, v_min_p, a_max_p, a_min_p, j_max_p, j_min_p, v_flag, vel_lim)
%function creates a profile using motor constraints given by:
% precentages of: velocity, acceleration, jerk given the Boundary condtions
% (v_max_p, v_min_p, a_max_p, a_min_p, j_max_p, j_min_p)
% and the values of velocity  and accelration at the start and end (v_s,
% v_f, a_s, a_f)
% the initial and final starting point should also be given: start_p, final_P
% v_flag logical variable definess wether the maximum velocity should be
% reached or not
% max values possible
% assert(v_f ~= v_s);
v_min = -10; v_max = 10;
a_min = -10; a_max = 10;
j_min = -10; j_max = 10;

%velocities
v_min = v_min*v_min_p;
v_max = v_max*v_max_p;
%accleration
a_min = a_min*a_min_p;
a_max = a_max*a_max_p;
% jerks
j_min = j_min*j_min_p;
j_max = j_max*j_max_p;
%=========================================================================%
%check wether final postion is before current postion

    if p_s < p_f
%          calculate double jerk impulse time
        T_s_j = min(sqrt((v_f- v_s)/j_max), a_max/j_max);

%          check if trajectory is possible
        if (T_s_j < a_max && p_f - p_s > T_s_j*(v_f + v_s)) || (T_s_j == a_max && p_f - p_s > (1/2*(v_s + v_f)*(T_j_s + abs(v_f - v_s)/a_max)))
            %if v_lim should reach v_max
            if v_flag == 1
                v_lim = v_max;
                disp("v_lim = v_max");
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
%                   Total time for the constant velocity
                T_v = (p_f - p_s)/v_max - T_a/2*(1+v_s/v_max) - T_d/2*(1+v_f/v_max);
                
                if T_v > 0
                    disp("maximum velocity reached");
                else
                    disp("maximum velcity not reached");
                end 
                T_d = round(T_d,2);
                T_a = round(T_a,2);
                T_v = round(T_v,2);
                T_j_1 = round(T_j_1,2);
                T_j_2 = round(T_j_2,2);
                v_lim = v_max;
                a_lim_a = a_max;
                a_lim_d = a_min;
                return 
%                 limit velocity does not reach max velocity
            else 
                disp("specified v_lim < v_max");
                disp("no constant velocity phase");
                T_j = a_max/j_max;
                T_j_1 = T_j;
                T_j_2 = T_j;
                T_v = 0;
                delta = (a_max^4/j_max^2)+2*(v_s^2+v_f^2)+a_max*(4*(p_f-p_s) - 2*a_max/j_max*(v_s+v_f));
                T_a = (a_max^2/j_max - 2*v_s+sqrt(delta))/(2*a_max);
                T_d = (a_max^2/j_max - 2*v_f+sqrt(delta))/(2*a_max);
                T_d = round(T_d,2);
                T_a = round(T_a,2);
                T_v = round(T_v,2);
                T_j_1 = round(T_j_1,2);
                T_j_2 = round(T_j_2,2);
                v_lim = vel_lim;
                a_lim_a = a_max;
                a_lim_d = a_min;
                return
            end 
        else
            disp("no possible trajectory")
            return
        end 
    
    end 
end

    
    
    
    