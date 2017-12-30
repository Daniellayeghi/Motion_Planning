%given the inputs for position start and final, velocity start and final,
%max and min constraints on acceleration

% calculate the new start and final position based on wether pos_1<pos_2
% and vel_1<vel_2;

pos_0 = 0;
pos_1 = 10;
vel_0 = 0;
vel_1 = 0;
vel_max = 5;
vel_min = -5;
acc_max = 10; 
acc_min = -10;
jerk_max = 30;
jerk_min = -30;
dec_point = 5;

rev = sign(pos_1-pos_0);

% new values for position 
p_f = rev*pos_1;% 
p_s = rev*pos_0; % 
% p_s = pos_0; 
% p_f = pos_1;
% velocity 
v_s = rev*vel_0;
v_f = rev*vel_1;
% v_s = vel_0;
% v_f = vel_1;
%   new constraints for velocity and accelration based on rev
v_max = ((rev+1)/2*vel_max + (rev-1)/2*vel_min);
v_min = ((rev+1)/2*vel_min + (rev-1)/2*vel_max);
a_max = ((rev+1)/2*acc_max + (rev-1)/2*acc_min);
a_min = ((rev+1)/2*acc_min + (rev-1)/2*acc_max);
j_max = ((rev+1)/2*jerk_max + (rev-1)/2*jerk_min);
j_min = ((rev+1)/2*jerk_min + (rev-1)/2*jerk_max);

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
%             [T_j_1, T_j_2, T_a, T_d, T_v, a_lim_a, a_lim_d, v_lim, j_max, j_min] = v_lim_s_traj(p_f, p_s, v_f, v_s, a_max, a_min, j_max, j_min, dec_point);
           
            T_j = round(a_max/j_max, dec_point);
            T_j_1 = T_j;
            T_j_2 = T_j;
            T_v = 0;
            delta = (a_max^4/j_max^2)+2*(v_s^2+v_f^2)+a_max*(4*(p_f-p_s) - 2*a_max/j_max*(v_s+v_f));
            T_a = round((a_max^2/j_max - 2*v_s+sqrt(delta))/(2*a_max), dec_point);
            T_d = round((a_max^2/j_max - 2*v_f+sqrt(delta))/(2*a_max), dec_point);

            %solve for velocity and accelration limits
                while T_a < 2*T_j || T_d < 2*T_j
%                     break
                    a_max = 0.70 * a_max;
                    if(a_max < 0.00001)
                        T_j = 0;
                        T_j_1 = T_j;
                        T_j_2 = T_j;
                    else
                        T_j = round(a_max/j_max, dec_point);
                        T_j_1 = T_j;
                        T_j_2 = T_j;
                    end 
                    T_v = 0;
                    delta = (a_max^4/j_max^2)+2*(v_s^2+v_f^2)+a_max*(4*(p_f-p_s) - 2*a_max/j_max*(v_s+v_f));
                    T_a = round((a_max^2/j_max - 2*v_s+sqrt(delta))/(2*a_max), dec_point);
                    T_d = round((a_max^2/j_max - 2*v_f+sqrt(delta))/(2*a_max), dec_point);

                    %solve for velocity and accelration limits
                    if T_a < 0 && T_d > 0
%                         break;
                        T_a = 0;
                        T_d = round(2*(p_f - p_s)/(v_f + v_s), dec_point);
                        T_j_2 = round((j_max*(p_f - p_s) - sqrt(j_max*(j_max*(p_f - p_s)^2+(v_f + v_s)^2*(v_f - v_s))))/(j_max*(v_f+v_s)), dec_point);

                    elseif T_a > 0 && T_d < 0 
                        T_d = 0; 
                        T_a = round(2*(p_f - p_s)/(v_f + v_s), dec_point);
                        T_j_1 = round((j_max*(p_f - p_s) - sqrt(j_max*(j_max*(p_f - p_s)^2+(v_f + v_s)^2*(v_f - v_s))))/(j_max*(v_f+v_s)), dec_point);
                    
                    end
%                     break;
%                     
                    disp(p_f);
                    disp(p_s);
                    disp(v_s);
                    disp(v_f);
                    disp(j_min);
                    disp(j_max);
                    disp(a_max);
                    disp(a_min);
                    disp(T_j)
                    disp(T_a);
                    disp(T_d);
                    disp("end")
                end
%             elseif T_a < 0 && T_d > 0
% 
%                 T_a = 0;
%                 T_d = round(2*(p_f - p_s)/(v_f - v_s), dec_point);
%                 T_j_2 = round((j_max*(p_f - p_s) - sqrt(j_max*(j_max*(p_f - p_s)^2+(v_f + v_s)^2*(v_f - v_s))))/(j_max*(v_f-v_s)), dec_point);
% 
%             elseif T_a > 0 && T_d < 0 
% 
%                 T_d = 0; 
%                 T_a = round(2*(p_f - p_s)/(v_f - v_s), dec_point);
%                 T_j_1 = round((j_max*(p_f - p_s) - sqrt(j_max*(j_max*(p_f - p_s)^2+(v_f + v_s)^2*(v_f - v_s))))/(j_max*(v_f-v_s)), dec_point);
        end 
            a_lim_a = j_max *T_j_1;
            a_lim_d = -j_max *T_j_2;
            v_lim = v_s+(T_a-T_j_1)*a_lim_a;
            j_max;
            j_min;
            T = T_a+T_d+T_v;
            

%   %  else 
%         disp("trajectory not possible");
%     end 


