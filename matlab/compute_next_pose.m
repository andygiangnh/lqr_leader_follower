%% the function for computing the controller -- velocity and turnrate 

function  [phi_d_next,x_d_next,y_d_next]=compute_next_pose(phi_d_current,x_d_current,y_d_current,velocity,turnrate,dt)

num_small_steps =100;

x_old=x_d_current;
y_old=y_d_current;
phi_old=phi_d_current;

for i=1:num_small_steps
    
     
        % Vehicle new position after one small step
        phi_new = phi_old+turnrate*dt/num_small_steps;        
        x_new = x_old+velocity*cos(phi_new)*dt/num_small_steps;
        y_new = y_old+velocity*sin(phi_new)*dt/num_small_steps;
        
        % update for iteration
        phi_old = phi_new;
        x_old = x_new;
        y_old = y_new;
end

phi_d_next=wrap(phi_new);
x_d_next=x_new;
y_d_next=y_new;