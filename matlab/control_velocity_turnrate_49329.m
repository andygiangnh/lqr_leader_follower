%% the function for computing the closed-loop controller -- velocity and turnrate 

function  [velocity_1,turnrate_1] = ...
    control_velocity_turnrate_49329(K_opt,V_0,Omega_0,deltatheta,deltaX,deltaY,phi)

    %% the current state
    j_last = size(deltatheta,2);
    X_current=[-1*(cos(phi)*deltaX(j_last) + sin(phi)*deltaX(j_last));
               sin(phi)*deltaY(j_last) - cos(phi)*deltaY(j_last);
               deltatheta(j_last)];

       
    % control value from state feedback
    U = -K_opt*X_current;
        
    %% get the velocity and turnrate values 
    velocity_1 = U(1) + V_0*cos(deltatheta(j_last));
    turnrate_1 = U(2) + Omega_0;
end
