%% 49329 Assignment 2 Part 2: robot moves along a straight line 
%% control to make the robot follow a straight line with constant speed

clc
clear all
close all

%% Parameter -- choose open-loop or closed-loop
closed_loop=1; % open-loop
%closed_loop=1; % closed-loop

time_period = 20; % simulation time period
dt = 0.02;   % sample time
total_steps = time_period/dt; % number of steps to simulate

% velocity and angular velocity of the desired trajectory
V_0 = 0.6; % velocity 
% Omega_0 = pi/6; % angular velocity

% noise on the velocity and angular velocity
V_noise = 0.1;   % uncertainty on velocity 
Omega_noise = 0.1;   % uncertainty on angular velocity 

% noise level on pose estimation
xy_noise = 0.05; % xy error in meter
theta_noise = 0.05; % orientation errot in radius

% for drawing of the robot heading
arrow_length = 0.1; % length of the arrow (for drawing robot orientation)

%% Design LQR control
A = [0 pi/6 0; -pi/6 0 0.6; 0 0 0];
B = [-1 0;0 0;0 -1];
C = eye(3);
D = 0;

Q = eye(3)*9;
R = eye(2);

[K_opt, P_opt, E_opt] = lqr(A,B,Q,R)


%% figure 1: axis and values for the vehicle motion

figure(1)

plot(0,0,'gd') % global coordinate
hold on;
axis('equal')
axis([-5 25 -10 10 ]);
xlabel('X position (m)')
ylabel('Y position (m)')
title('Trajectories of the robot. Red: desired, Blue: actual')

%% set of the objet properties
%% initialisation: plot the first point (t=0)

% time step
t(1) = 0;

% initial position of robot
x(1) = 0;
y(1) = 0;
phi(1) = 0;
Omega_0(1) = pi/6; % angular velocity

% initial position of the desired robot position
x_d(1) = 0;
y_d(1) = 0;
phi_d(1) = 0;

% draw the vehicle position and orientation
quiver(x(1),y(1), arrow_length*cos(phi(1)), arrow_length*sin(phi(1)), 0, 'Color', 'b') 
quiver(x_d(1),y_d(1), arrow_length*cos(phi_d(1)), arrow_length*sin(phi_d(1)), 0, 'Color', 'r')

%% difference between the desired and actual pose

deltatheta(1) = wrap(phi(1)-phi_d(1));   % orientation 
deltaX(1) = x(1)-x_d(1);
deltaY(1) = y(1)-y_d(1);

% % add noise on the differences
% deltatheta(1)=deltatheta(1)+randn(1,1)*theta_noise;
% deltaX(1)=deltaX(1)+randn(1,1)*xy_noise;
% deltaY(1)=deltaY(1)+randn(1,1)*xy_noise;

%% start the simulation -- get the control values from closed-loop
for j=1:total_steps    
       t(j+1) = j*dt;    % used just to plot the velocity curve
        
       % adjust the desired trajectory by changing the desired angular
       % velocity
       Omega_0(j) = 0.15*sin(0.2*t(j)) - 0.3*cos(t(j));
        
        % get the control-- velocity and turnrate
        [velocity_1,turnrate_1] = ...
            control_velocity_turnrate_49329(K_opt, V_0, Omega_0(j),...
            deltatheta, deltaX, deltaY, phi(j));        
        
  % pause    
  velocity_1=velocity_1+randn(1,1)*V_noise;
  turnrate_1=turnrate_1+randn(1,1)*Omega_noise;
  
        % record for drawing
        vel_1(j)=velocity_1;
        turn_1(j)=turnrate_1;
        
       % compute the next pose using the velocity and turnrate (discretization with small steps) 
        
        [phi(j+1),x(j+1),y(j+1)]=compute_next_pose(phi(j),x(j),y(j),velocity_1,turnrate_1,dt);
        
        % draw the position and orientation
        quiver(x(j+1),y(j+1), arrow_length*cos(phi(j+1)), arrow_length*sin(phi(j+1)), 0, 'Color', 'b')
                
        % the trajectory of wheelchair 1
        xyphi = [x;y;phi];
        
             
        % do not add noises 
        velocity_d = V_0;          
        turnrate_d = Omega_0(j);  
        
        % add noises 
        % velocity_d = V_0 + randn(1,1)*V_noise;          
        % turnrate_d = Omega_0 + randn(1,1)*Omega_noise;  
         
        % record for drawing
        vel_d(j)=velocity_d;
        turn_d(j)=turnrate_d;
        
        % compute the next pose using the velocity and turnrate
        % (discretization with small steps)
        [phi_d(j+1),x_d(j+1),y_d(j+1)]=compute_next_pose(phi_d(j),x_d(j),y_d(j),velocity_d,turnrate_d,dt);
        
        % drawing     
        quiver(x_d(j+1),y_d(j+1), arrow_length*cos(phi_d(j+1)), arrow_length*sin(phi_d(j+1)), 0, 'Color', 'r')
        
        
        % difference between actural and desired pose
        deltatheta(j+1) = wrap(phi(j+1)-phi_d(j+1));
        deltaX(j+1) = x(j+1)-x_d(j+1);
        deltaY(j+1) = y(j+1)-y_d(j+1);
        
% add noise on the relative pose -- simulate the localization result
        deltatheta(j+1)=deltatheta(j+1)+randn(1,1)*theta_noise;
        deltaX(j+1)=deltaX(j+1)+randn(1,1)*xy_noise;
        deltaY(j+1)=deltaY(j+1)+randn(1,1)*xy_noise;
    
    %drawnow

end 

%% set the velocity the same as previous step -- only for drawing

vel_1(total_steps+1) = vel_1(total_steps);  
vel_d(total_steps+1) = vel_d(total_steps);  
turn_1(total_steps+1) = turn_1(total_steps);
turn_d(total_steps+1) = turn_d(total_steps);

% Plot the whole trajectory (all points)

% actual robot position
plot(x(:),y(:),'+b')  
% desired robot position
plot(x_d(:),y_d(:),'or') 

hold off

%% features figure (2) vels1/vels2

figure(2)
hold off
clf
hold on
%axis([0 total_steps -0.5 0.5]);
xlabel('Time step')
ylabel('Velocity (m/sec) and angular velocity (rad/sec)')
title('Velocity and angular velocity')

%% Plot velocity and turnrate for W1 and W2 to show the tracking result

figure(2)
plot(t(1:end), vel_1(1:end),'k-')
plot(t(1:end), vel_d(1:end),'r-')

plot(t(1:end), turn_1(1:end),'b:')
plot(t(1:end), turn_d(1:end),'r:')

legend('actual velocity','desired velocity','actual angular velocity','desired angular velocity')

hold off


%%  figure (3): relative distance error deltaX/deltaY

% Plot the error in x y phi

figure(3)
  subplot(3,1,1), plot(t,deltaX(:),'-r');
  xlabel('Time (sec)')
ylabel('x error (m)')
  
    subplot(3,1,2), plot(t,deltaY(:),'-b');
    xlabel('Time (sec)')
ylabel('y error (m)')
    
    subplot(3,1,3), plot(t,deltatheta(:),'-k');
    xlabel('Time (sec)')
ylabel('phi error (rad)')

hold off
