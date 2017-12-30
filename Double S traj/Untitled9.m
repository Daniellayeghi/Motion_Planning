for i = 1:6 
    if i ~= row_max 
        alpha = 1/2;
        beta = 1/2;
        a_lim_a(i,1)=acc_max(i,1);
        a_lim_d(i,1)=acc_min(i,1); 
%         v_lim(i,1) = (pos(i,2) - pos(i,1))/((1-alpha)*T(i,1));
%         a_lim_a(i,1) = (pos(i,2) - pos(i,1))/(alpha*(1-alpha)*(1-beta)*T(i,1)^2);
%         a_lim_d(i,1) = -a_lim_a(i,1); 
%         j_max(i,1) = (pos(i,2) - pos(i,1))/(alpha^2*beta*(1-alpha)*(1-beta)*T(i,1)^3);
%         j_min(i,1) = -j_max(i,1);
        v_lim(i,1) = (-a_lim_a(i,1)^2+a_lim_a(i,1)*j_max(i,1)*max_T - sqrt(a_lim_a(i,1)*(-4*(pos(i,2) - pos(i,1))*j_max(i,1)^2+a_lim_a(i,1)*((a_lim_a(i,1)-j_max(i,1)*max_T)^2))))/(2*j_max(i,1));
        T_a(i,1) = round(T(i,1)*alpha,2);
        T_j(i,1) = round(T_a(i,1)*beta,2);
        T_j_1(i,1) = a_lim_a(i,1)/j_max(i,1);
        T_j_2(i,1) = T_j_1(i,1);
        T(i,1) = max_T;
        T_v(i,1) = T(i,1)-(T_a(i,1) + T_d(i,1)); 
    end 
end 