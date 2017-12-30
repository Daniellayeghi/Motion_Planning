% regen trajectories based on maximum time
% assume initial and final velocity as 0
a_lim = zeros(6,1);
for i = 1:6
    if i ~= row_max
        a_lim(i,1) = 2*(pos(i,2) - pos(i,1))+sqrt(4*(pos(i,2) - pos(i,1))^2)/(max_T)^2;
        if a_lim_a(i,1)<a_lim(i,1)
            a_lim_a(i,1) = a_lim(i,1);
        end
        v_lim(i,1) = 1/2*(a_lim_a(i,1)*max_T - sqrt(a_lim_a(i,1)^2*max_T^2 - 4*a_lim_a(i,1)*(pos(i,2)-pos(i,1))));
        T_a(i,1) = v_lim(i,1)/a_lim_a(i,1);
        T_d(i,1) = T_a(i,1);
    end 
end 

         
 
 