function [A,Bmv] = myStateJacobian(x,u,t,ax,ay,mu)
% m = 1650 + 180; % Sprung mass and unsprung mass
% g = 9.81;
% Iz = 3234;
% Iw = 0.9; % wheel inertia
% Lw = 1.6; % wheel base
% lf = 1.4;
% lr = 1.65;
% % cornering stiffness for a single wheel [N/rad]
% Cf = 1205.3*180/pi; % Fzf = 4856*2 N
% Cr = 1122.7*180/pi; % Fzr = 4140*2 N
% reff = 0.353;
% h = 0.53;
% 
% % Aero Dynamics
% Area = 2.8;
% rho = 1.206;
% Cd = 0.3;
% 
% % States
% beta = x(1); % [rad]
% r = x(2); % [rad/s]
% yaw = x(3); % [rad]
% x_glo = x(4); % [m]
% y_glo = x(5); % [m]
% v = x(6); % [m/s]
% disp(beta)
% if abs(v) < 1e-3
%     v = 16.7;
% end
% 
% % Inputs
% delta = u(1); % [rad] 
% T = u(2); % [Nm]
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% A Matrix %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% A = [-2*(Cf+Cr)/(m*v)       -2*(Cf*lf-Cr*lr)/(m*v^2)-1    0                 0   0   (4*r*(Cf*lf-Cr*lr)-v*2*Cf*(delta-beta)+v*2*Cr*beta)/(m*v^3);
%     2*(-Cf*lf+Cr*lr)/Iz     2*(-Cf*lf^2-Cr*lr^2)/(Iz*v)   0                 0   0   r*2*(Cf*lf^2+Cr*lr^2)/(Iz*v^2);
%     0                       1                             0                 0   0   0;
%     -v*sin(beta+yaw)        0                             -v*sin(beta+yaw)  0   0   cos(beta+yaw);
%     v*cos(beta+yaw)         0                             v*cos(beta+yaw)   0   0   sin(beta+yaw);
%     0                       0                             0                 0   0   -Area*rho*Cd*v/(m+4*Iw/(reff^2))];
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% B Matrix %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Bmv = [2*Cf/(m*v) 0;
%     2*Cf*lf/Iz  0;
%     0           0;
%     0           0;
%     0           0;
%     0           1/(m*reff+4*Iw/reff)];

[A,Bmv]=linearize_model(x,u,t,ax,ay,mu);