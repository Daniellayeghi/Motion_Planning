% symmetric double s trajectory generation
h = zeros(6,1);
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
    h(i,1) = pos(i,2) - pos(i,1);
end
[max_dist, joint] = max(h);

if (vel_max(joint) - vel(joint,2))*j_max(joint)  < acc_max(joint,1)^2
    disp("acc_max not reached");
    T_j_1(joint,1) = round(sqrt((vel_max(joint,1) - vel(joint,1))/j_max(joint,1)),2);
    T_a(joint,1) = round(2*T_j_1(joint,1),2);
else
    T_j_1(joint,1) = round(acc_max(joint,1)/j_max(joint,1),2);
    T_a(joint,1) = round(T_j_1(joint,1)+(vel_max(joint,1)-vel(joint,1))/acc_max(joint,1),2);
end 

if (vel_max(joint) - vel(joint,2))*j_max(joint)  < acc_max(joint,1)^2
    disp("a_min not reached");
    T_j_2(joint) = round(sqrt((vel_max(joint) - vel(joint,2))/j_max(joint)),2);
    T_d(joint) = 2*T_j_2(joint);
else 
    T_j_2(joint) = round(acc_max(joint,1)/j_max(joint),2);
    T_d(joint) = round(T_j_2(joint)+(vel_max(joint)-vel(joint,2))/acc_max(joint,1),2);
end
T_v(joint) = round((pos(joint,2) - pos(joint,1))/vel_max(joint) - T_a(joint)/2*(1+vel(joint,1)/vel_max(joint)) - T_d(joint)/2*(1+vel(joint,2)/vel_max(joint)),2);

while T_v(joint) <=0 
    disp("reducing")
    flag = true;
   vel_max(joint)= 0.95*vel_max(joint); 
   if (vel_max(joint) - vel(joint,2))*j_max(joint)  < acc_max(joint,1)^2
    disp("acc_max not reached");
    T_j_1(joint,1) = round(sqrt((vel_max(joint,1) - vel(joint,1))/j_max(joint,1)),2);
    T_a(joint,1) = round(2*T_j_1(joint,1),2);
    else
        T_j_1(joint,1) = round(acc_max(joint,1)/j_max(joint,1),2);
        T_a(joint,1) = round(T_j_1(joint,1)+(vel_max(joint,1)-vel(joint,1))/acc_max(joint,1),2);
    end 

    if (vel_max(joint) - vel(joint,2))*j_max(joint)  < acc_max(joint,1)^2
        disp("a_min not reached");
        T_j_2(joint) = round(sqrt((vel_max(joint) - vel(joint,2))/j_max(joint)),2);
        T_d(joint) = round(2*T_j_2(joint),2);
    else 
        T_j_2(joint) = round(acc_max(joint,1)/j_max(joint),2);
        T_d(joint) = round(T_j_2(joint)+(vel_max(joint)-vel(joint,2))/acc_max(joint,1),2);
    end
    T_v(joint) = round((pos(joint,2) - pos(joint,1))/vel_max(joint) - T_a(joint)/2*(1+vel(joint,1)/vel_max(joint)) - T_d(joint)/2*(1+vel(joint,2)/vel_max(joint)),2);
   
end
    T(joint) = T_d(joint)+T_a(joint)+T_v(joint);
    a_lim_a(joint) = j_max(joint) *T_j_1(joint);
    a_lim_d(joint) = -j_max(joint) *T_j_2(joint);
    v_lim(joint) = vel(joint,1)+(T_a(joint)-T_j_1(joint))*a_lim_a(joint);
%     j_max(joint);
%     j_min(joint);

    max_T = T(joint);
    [row_max, ~] = ind2sub(size(T), joint);
    T_a(joint,1) = round(T_a(joint,1),2);
    T_d(joint,1) = round(T_d(joint,1),2);
    T(joint,1) = round(T(joint,1),2);
    T_j_1(joint,1) = round(T_j_1(joint,1),2);
    T_j_2(joint,1) = round(T_j_2(joint,1),2);
    T_v(joint,1) = round(T_v(joint,1),2);
    
    
    for i = 1:6 
%         if i ~= joint
            v_lim(i,1) = h(i,1)/(max_T-T_a(joint,1));
            a_lim_a(i,1) = h(i,1)/((max_T-T_a(joint,1))*(T_a(joint,1)-T_j_1(joint,1)));
            a_lim_d(i,1) = -a_lim_a(i,1);
            j_max(i,1) = h(i,1)/((max_T-T_a(joint,1))*(T_a(joint,1)-T_j_1(joint,1))*T_j_1(joint,1));
            j_min(i,1) = -j_max(i,1);
            T_j_1(i,1) = T_j_1(joint,1);
            T_j_2(i,1) = T_j_2(joint,1);
            T_a(i,1) = T_a(joint,1);
            T_d(i,1) = T_d(joint,1);
            T(i,1) = T(joint);
            T_v(i,1) = T(i,1) - 2*T_a(i,1);
%         end 
    end           