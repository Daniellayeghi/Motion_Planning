%given the inputs for position start and final, velocity start and final,
%max and min constraints on acceleration

% calculate the new start and final position based on wether pos_1<pos_2
% and vel_1<vel_2;

% pos = zeros(6,2);
% vel = zeros(6,2);
% vel_max = zeros(6,1);
% vel_min = zeros(6,1);
% acc_max = zeros(6,1);
% acc_min = zeros(6,1);
% j_max = zeros(6,1);
% j_min = zeros(6,1);
%==========================================================================

T_j_1 = zeros(6,1);
T_j_2 = zeros(6,1);
T_a = zeros(6,1);
T_d = zeros(6,1);
T_v = zeros(6,1);
T_j = zeros(6,1);
T = zeros(6,1);
v_lim = zeros(6,1);
a_lim_a = zeros(6,1);
a_lim_d = zeros(6,1);
rev = zeros(6,1);
scale = zeros(6,1);
dec_point = 2;


for i = 1:6 
    rev(i,1) = sign(pos(i,2)-pos(i,1));    

    pos(i,1) = rev(i,1)*pos(i,1);
    pos(i,2) = rev(i,1)*pos(i,2);
    vel(i,1) = rev(i,1)*vel(i,1);
    vel(i,2) = rev(i,1)*vel(i,2);
    
    vel_max(i,1) = ((rev(i,1)+1)/2*vel_max(i,1) + (rev(i,1)-1)/2*vel_min(i,1));
    vel_min(i,1) = ((rev(i,1)+1)/2*vel_min(i,1) + (rev(i,1)-1)/2*vel_max(i,1));
    acc_max(i,1) = ((rev(i,1)+1)/2*acc_max(i,1) + (rev(i,1)-1)/2*acc_min(i,1));
    acc_min(i,1) = ((rev(i,1)+1)/2*acc_min(i,1) + (rev(i,1)-1)/2*acc_max(i,1));
    j_max(i,1) = ((rev(i,1)+1)/2*j_max(i,1) + (rev(i,1)-1)/2*j_min(i,1));
    j_min(i,1) = ((rev(i,1)+1)/2*j_min(i,1) + (rev(i,1)-1)/2*j_max(i,1));
    
    
        if (vel_max(i) - vel(i,2))*j_max(i)  < acc_max(i,1)^2
            disp("acc_max not reached");
            T_j_1(i,1) = sqrt((vel_max(i,1) - vel(i,1))/j_max(i,1));
            T_a(i,1) = 2*T_j_1(i,1);
            
        else
            T_j_1(i,1) = acc_max(i,1)/j_max(i,1);
            T_a(i,1) = T_j_1(i,1)+(vel_max(i,1)-vel(i,1))/acc_max(i,1);
            
        end 
        
         if (vel_max(i) - vel(i,2))*j_max(i)  < acc_max(i,1)^2
            disp("a_min not reached");
            T_j_2(i) = sqrt((vel_max(i) - vel(i,2))/j_max(i));
            T_d(i) = 2*T_j_2(i);
        else 
            T_j_2(i) = acc_max(i,1)/j_max(i);
            T_d(i) = T_j_2(i)+(vel_max(i)-vel(i,2))/acc_max(i,1);
         end
        
        T_v(i) = (pos(i,2) - pos(i,1))/vel_max(i) - T_a(i)/2*(1+vel(i,1)/vel_max(i)) - T_d(i)/2*(1+vel(i,2)/vel_max(i));
        
        if T_v(i) > 0
%           computation is done
            disp("maximum velocity reached");
            T_d(i) = round(T_d(i), dec_point);
            T_a(i) = round(T_a(i), dec_point);
            T_v(i) = round(T_v(i), dec_point);
            T_j_1(i) = round(T_j_1(i), dec_point);
            T_j_2(i) = round(T_j_2(i), dec_point);
            v_lim(i) = vel_max(i);
            a_lim_a(i) = acc_max(i,1);
            a_lim_d(i) = acc_min(i);
        
        else
            disp("maximum velocity not reached");
%             [T_j_1, T_j_2, T_a, T_d(i), T_v, a_lim_a, a_lim_d, v_lim, j_max, j_min] = v_lim_s_traj(pos(i,2), p_s, vel(i,2), vel(i,1), acc_max(i,1), a_min, j_max, j_min, dec_point);
           
            T_j(i) = round(acc_max(i,1)/j_max(i), dec_point);
            T_j_1(i) = T_j(i);
            T_j_2(i) = T_j(i);
            T_v(i) = 0;
            delta = (acc_max(i,1)^4/j_max(i)^2)+2*(vel(i,1)^2+vel(i,2)^2)+acc_max(i,1)*(4*(pos(i,2)-pos(i,1)) - 2*acc_max(i,1)/j_max(i)*(vel(i,1)+vel(i,2)));
            T_a(i) = round((acc_max(i,1)^2/j_max(i) - 2*vel(i,1)+sqrt(delta))/(2*acc_max(i,1)), dec_point);
            T_d(i) = round((acc_max(i,1)^2/j_max(i) - 2*vel(i,2)+sqrt(delta))/(2*acc_max(i,1)), dec_point);
            
            while T_a(i) < 2*T_j(i) || T_d(i) < 2*T_j(i)
                
%                     break
                    acc_max(i,1) = 0.70 * acc_max(i,1);
                    if(acc_max(i,1) < 0.001)
                        T_j(i) = 0;
                        T_j_1(i) = T_j(i);
                        T_j_2(i) = T_j(i);
                    else
                        T_j(i) = round(acc_max(i,1)/j_max(i), dec_point);
                        T_j_1(i) = T_j(i);
                        T_j_2(i) = T_j(i);
                    end 
                    T_v(i) = 0;
                    delta = (acc_max(i,1)^4/j_max(i)^2)+2*(vel(i,1)^2+vel(i,2)^2)+acc_max(i,1)*(4*(pos(i,2)-pos(i,1)) - 2*acc_max(i,1)/j_max(i)*(vel(i,1)+vel(i,2)));
                    T_a(i) = round((acc_max(i,1)^2/j_max(i) - 2*vel(i,1)+sqrt(delta))/(2*acc_max(i,1)), dec_point);
                    T_d(i) = round((acc_max(i,1)^2/j_max(i) - 2*vel(i,2)+sqrt(delta))/(2*acc_max(i,1)), dec_point);

                    %solve for velocity and accelration limits
                    if T_a(i) < 0 && T_d(i) > 0
%                         break;
                        T_a(i) = 0;
                        T_d(i) = round(2*(pos(i,2) - pos(i,1))/(vel(i,2) + vel(i,1)), dec_point);
                        T_j_2(i) = round((j_max(i)*(pos(i,2) - pos(i,1)) - sqrt(j_max(i)*(j_max(i)*(pos(i,2) - pos(i,1))^2+(vel(i,2) + vel(i,1))^2*(vel(i,2) - vel(i,1)))))/(j_max(i)*(vel(i,2)+vel(i,1))), dec_point);

                    elseif T_a(i) > 0 && T_d(i) < 0 
                        T_d(i) = 0; 
                        T_a(i) = round(2*(pos(i,2) - pos(i,1))/(vel(i,2) + vel(i,1)), dec_point);
                        T_j_1(i) = round((j_max(i)*(pos(i,2) - pos(i,1)) - sqrt(j_max(i)*(j_max(i)*(pos(i,2) - pos(i,1))^2+(vel(i,2) + vel(i,1))^2*(vel(i,2) - vel(i,1)))))/(j_max(i)*(vel(i,2)+vel(i,1))), dec_point);
                    end
            end
        end 

        T(i) = T_d(i)+T_a(i)+T_v(i);
        a_lim_a(i) = j_max(i) *T_j_1(i);
        a_lim_d(i) = -j_max(i) *T_j_2(i);
        v_lim(i) = vel(i,1)+(T_a(i)-T_j_1(i))*a_lim_a(i);
        j_max(i);
        j_min(i);
        
        [max_T, joint] = max(T);
        [row_max, ~] = ind2sub(size(T), joint);
        

end 
        for j = 1 : 6
            
%                 T(j,1) = max_T;
%                 a_lim_a(j,1) = acc_max(j,1);
%                 a_lim_d(j,1) = acc_min(j,1);
                scale(j,1) = round(T(j,1)/max_T,dec_point);
        end 
