% plot trajectories
iters = round(t_pq_sync*100);
x = zeros(iters,6);
v = zeros(iters,6);
a = zeros(iters,6);
j = zeros(iters,6);
time = zeros(iters,6);
t_pq_sync = round(t_pq_sync,2);
t_pl_sync = round(t_pl_sync,2);
t_ls_sync = round(t_ls_sync,2);

for i = 1:6
    for ts = 0:0.01:round(t_pq_sync,2)
        if ts<=t_pl_sync
             x(round(ts*100)+1,i) = position(i,1)+sync_jerk(i,1)/(2*sync_w(i,1)^3)*((sync_w(i,1)*ts)^2-2*(1-cosd(sync_w(i,1)*ts)));
             v(round(ts*100)+1,i) = sync_jerk(i,1)/(sync_w(i,1)^2)*((sync_w(i,1)*ts)-sind(sync_w(i,1)*ts)); 
             a(round(ts*100)+1,i) = sync_jerk(i,1)/(sync_w(i,1))*(1-cosd(sync_w(i,1)*ts));
             j(round(ts*100)+1,i) = sync_jerk(i,1)*sind(sync_w(i,1)*ts); 
        end 
        if ts>t_pl_sync && ts<=t_pl_sync+t_ls_sync
            x(round(ts*100)+1,i) = 1/2*sync_acc(i,1)*t_pl_sync(ts-t_pl_sync);
            v(round(ts*100)+1,i) = v(round(ts*100),i)+1/2*sync_acc(i,1)*t_pl_sync; 
            a(round(ts*100)+1,i) = 0;
            j(round(ts*100)+1,i) = 0; 
        end 
        if ts>t_pl_sync+t_ls_sync && ts<=t_pq_sync
            x(round(ts*100)+1,i) = sync_jerk(i,1)/(2*sync_w(i,1)^3)*((sync_w(i,1)*(ts-(t_pl_sync+t_ls_sync))^2-2*(1-cosd(sync_w(i,1)*(ts-(t_pl_sync+t_ls_sync))))));
            v(round(ts*100)+1,i) = v(round(ts*100),i)+sync_jerk(i,1)/(sync_w(i,1)^2)*((sync_w(i,1)*(ts-(t_pl_sync+t_ls_sync)))-sind(sync_w(i,1)*(ts-(t_pl_sync+t_ls_sync)))); 
            a(round(ts*100)+1,i) = -sync_jerk(i,1)/(sync_w(i,1))*(1-cosd(sync_w(i,1)*ts));
            j(round(ts*100)+1,i) = -sync_jerk(i,1)*sind(sync_w(i,1)*ts);
        end
        time(round(ts*100)+1,i) = ts;
    end
    
end


            
           
        
            
        
            