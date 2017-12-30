%Trigonometric jerk bounded trajectories
%inputs max acceleration, max velocity, max constant velocity;
%initial and final positions

% position = zeros(6,2);
% max_acc;
% max_jerk;

%time segments lower bounds 
t_pl_l = zeros(6,1);
t_ls_l = zeros(6,1);
t_pq_l = zeros(6,1);

%position segments upper bounds
dist_pl_u = zeros(6,1);
dist_sq_u = zeros(6,1);

%segments distance and velocity
dist_pq = zeros(6,1);
dist_pl = zeros(6,1);
dist_ls  = zeros(6,1);

%segments constraints
acc_pl = zeros(6,1);
jerk_pl = zeros(6,1);
% v_ls = zeros(6,1);

% synced values
% t_pl_sync;
% t_ls_sync;
sync_ratio = zeros(6,1);
sync_acc = zeros(6,1);
sync_jerk = zeros(6,1);
sync_vel = zeros(6,1);
sync_w = zeros(6,1);
w_pl = zeros(6,1);



for i = 1:6
    % calculate the maximum time for the accelration phase
    t_pl_l(i,1) = round(max_acc(i,1)/max_jerk(i,1) * pi, 2);
    % required displacment for acceleration phase
    dist_pl_u(i,1) = 1/4*max_acc(i,1)*((t_pl_l(i,1))^2);
    dist_pq(i,1) = position(i,2) - position(i,1);
    
    if abs(dist_pq(i,1))<= abs(2*dist_pl_u(i,1))
        dist_pl(i,1) = dist_pq(i,1)/2;
        acc_pl(i,1) = 4*dist_pl(i,1)/((t_pl_l(i,1))^2);
        jerk_pl(i,1) = acc_pl(i,1)*pi/t_pl_l(i,1);
        %update accleration time phase
        t_pl_l(i,1) = round(acc_pl(i,1)/jerk_pl(i,1)*pi,2);
        t_ls_l(i,1) = 0;
    else
        dist_pl(i,1) = dist_pl_u(i,1);
        dist_ls(i,1) = dist_pq(i,1) - 2*dist_pl_u(i,1);
        v_ls(i,1) = acc_max * t_pl_l(i,1);
        t_ls_l(i,1) = round(dist_ls(i,1)/v_ls(i,1),2);
    end
    t_pq_l(i,1) = round(2*t_pl_l(i,1)+t_ls_l(i,1),2);

end 
    t_pl_sync = max(t_pl_l);
    t_ls_sync = max(t_ls_l);
    t_pq_sync = 2*(t_pl_sync)+t_ls_sync;
for i = 1:6    
    sync_ratio(i,1) = t_pl_l(i,1)/t_pl_sync;
    sync_acc(i,1) = 4*dist_pl(i,1)/t_pl_sync;
    w_pl(i,1) = 2*max_jerk(i,1)/(max_acc(i,1));
    sync_w(i,1) = sync_ratio(i,1)*w_pl(i,1);
    sync_jerk(i,1) = sync_w(i,1)*sync_acc(i,1)/2;
    sync_vel(i,1) = sync_acc(i,1)*pi/sync_w(i,1);
end 

    
   
    
        
    
    
    
    
    
    
    
    
        
        
        
    
    
    



        