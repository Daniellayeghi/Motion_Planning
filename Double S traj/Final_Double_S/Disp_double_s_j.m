function [x,v,a,j,time] = Disp_double_s_j (pos, vel, T_j_1, T_j_2, T_a, T_d, a_lim_a, a_lim_d, v_lim, j_max, j_min, T, T_v, rev, scale,row_max,max_T)

    
%% assume sampling at 100Hz in complete trajectory
    iters = round(max(T)*100);
% 
%% arrays t hold position and it's derivatives
    x = zeros(iters,6);
    v = zeros(iters,6);
    a = zeros(iters,6);
    j = zeros(iters,6);
    time = zeros(iters,6);
%  
% =========================================================================    

    for i = 1:6
%      assuming update rate of 0.01s

% arrays t hold position and it's derivatives
            for ts = 0:0.01:max_T
                if ts <= T_j_1(i,1)
                     x(round(ts*100)+1,i) = pos(i,1) + vel(i,1)*ts +j_max(i,1)*(ts^3/6);
                     v(round(ts*100)+1,i) = vel(i,1) + j_max(i,1)*(ts^2/2);
                     a(round(ts*100)+1,i) = j_max(i,1) *ts;
                     j(round(ts*100)+1,i) = j_max(i,1);
                end 

                if ts > T_j_1(i,1) && ts <= T_a(i,1) - T_j_1(i,1)
                     x(round(ts*100)+1,i) = pos(i,1) + vel(i,1)*ts + a_lim_a(i,1)/6*(3*ts^2-3*T_j_1(i,1)*ts+T_j_1(i,1)^2);
                     v(round(ts*100)+1,i) = vel(i,1) + a_lim_a(i,1)*(ts-T_j_1(i,1)/2);
                     a(round(ts*100)+1,i) = j_max(i,1) * T_j_1(i,1);
                     j(round(ts*100)+1,i) = 0;
                end 

                if ts > T_a(i,1) - T_j_1(i,1) && ts <= T_a(i,1)
                    x(round(ts*100)+1,i) = pos(i,1) +(v_lim(i,1)+vel(i,1))*T_a(i,1)/2-v_lim(i,1)*(T_a(i,1)-ts) - j_min(i,1)*(T_a(i,1)-ts)^3/6;
                    v(round(ts*100)+1,i) = v_lim(i,1) + j_min(i,1)*(T_a(i,1)-ts)^2/2;
                    a(round(ts*100)+1,i) = -j_min(i,1)*(T_a(i,1) - ts);
                    j(round(ts*100)+1,i) = j_min(i,1);

                end 
%                  constant velocity phase
                if ts > T_a(i,1) && ts <= T_a(i,1) + T_v(i,1) 
                   x(round(ts*100)+1,i) = pos(i,1)+(v_lim(i,1)+vel(i,1))*T_a(i,1)/2+v_lim(i,1)*(ts-T_a(i,1));
                   v(round(ts*100)+1,i) = v_lim(i,1);
                   a(round(ts*100)+1,i) = 0;
                   j(round(ts*100)+1,i) = 0;
                end 
%                  deceleration phase
                if ts > T_a(i,1) + T_v(i,1)  && ts <= T(i,1) - T_d(i,1) + T_j_2(i,1)
                    x(round(ts*100)+1,i) = pos(i,2) - (v_lim(i,1) + vel(i,2))*T_d(i,1)/2+v_lim(i,1)*(ts-T(i,1)+T_d(i,1)) - j_max(i,1)*(ts-T(i,1)+T_d(i,1))^3/6;
                    v(round(ts*100)+1,i) = v_lim(i,1) - j_max(i,1)*(ts-T(i,1)+T_d(i,1))^2/2;
                    a(round(ts*100)+1,i) = -j_max(i,1)*(ts-T(i,1)+T_d(i,1));
                    j(round(ts*100)+1,i) = j_min(i,1);
                end 

                if ts > T(i,1) - T_d(i,1) + T_j_2(i,1) && ts <= T(i,1)-T_j_2(i,1)
                    x(round(ts*100)+1,i) = pos(i,2) - (v_lim(i,1)+vel(i,2))*T_d(i,1)/2+v_lim(i,1)*(ts-T(i,1)+T_d(i,1))+a_lim_d(i,1)/6*(3*(ts-T(i,1)+T_d(i,1))^2 - 3*T_j_2(i,1)*(ts-T(i,1)+T_d(i,1))+T_j_2(i,1)^2);
                    v(round(ts*100)+1,i) = v_lim(i,1) +a_lim_d(i,1)*(ts-T(i,1)+T_d(i,1)-T_j_2(i,1)/2);
                    a(round(ts*100)+1,i) = -j_max(i,1)*T_j_2(i,1);
                    j(round(ts*100)+1,i) = 0;
                end 

                if ts > T-T_j_2(i,1) 
                    x(round(ts*100)+1,i) = pos(i,2) - vel(i,2)*(T(i,1)-ts) - j_max(i,1)*(T(i,1)-ts)^3/6;
                    v(round(ts*100)+1,i) = vel(i,2)+ j_max(i,1)*(T(i,1)-ts)^2/2;
                    a(round(ts*100)+1,i) = -j_max(i,1)*(T(i,1)-ts);
                    j(round(ts*100)+1,i) = j_max(i,1);
                end
                j(round(ts*100)+1,i) = rev(i,1) * j(round(ts*100)+1,i);

                
                time(round(ts*100)+1,i) = ts;
                x(round(ts*100)+1,i) = rev(i,1) * x(round(ts*100)+1,i);
                v(round(ts*100)+1,i) = rev(i,1) * v(round(ts*100)+1,i);
                a(round(ts*100)+1,i) = rev(i,1) * a(round(ts*100)+1,i);
                
            end
    end

    
%         subplot(2,2,1);
%         hold on;
%         plot(time,x(:,i));
%         legend("Postion");
%         
%         subplot(2,2,2);
%         hold on;
%         plot(time,v(:,i));
%         legend("Velocity");
%         
%         subplot(2,2,3);
%         hold on;
%         plot(time,a(:,i));
%         legend("Acceleration");
%         
%         subplot(2,2,4);
%         hold on;
%         plot(time,j(:,i));
%         legend("Jerk");
        
    return
end 