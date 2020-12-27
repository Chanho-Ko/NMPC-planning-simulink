function z = nonlinVehicleModel_com1(x,u,t, ax, ay, v, mu, s)
% These parameters are based on the model of E-class sedan
% from CarSim2017.1

m = 1650 + 180; % Sprung mass and unsprung mass
g = 9.81;
Iz = 3234;
Iw = 0.9; % wheel inertia
Lw = 1.6; % Track width
lf = 1.4;
lr = 1.65;
reff = 0.353;
h = 0.53;

% norminal cornering stiffness [N/rad]
Cf = (1305.3)*180/pi; % Fzf = 4856*2 N
Cr = (1122.7)*180/pi; % Fzr = 4140*2 N


% Aero Dynamics
A = 2.8;
rho = 1.206;
Cd = 0.3;

% States
beta = x(1); % [rad]
r = x(2); % [rad/s]
yaw = x(3); % [rad]
x_glo = x(4); % [m]
y_glo = x(5); % [m]
if abs(v) < 1e-3
    v = 1e-3;
end

% Input
delta = u(1); % [rad]

% Vertical Forces
Fz_nor = [4856 4856 4140 4140];
Fzfl = (g*lr/2-ax*h/2-1*ay*lr*h/Lw+ax*ay*h^2/g/Lw)*m/(lr+lf);
Fzfr = (g*lr/2-ax*h/2+1*ay*lr*h/Lw-ax*ay*h^2/g/Lw)*m/(lr+lf);
Fzrl = (g*lf/2+ax*h/2-1.2*ay*lf*h/Lw-ax*ay*h^2/g/Lw)*m/(lr+lf);
Fzrr = (g*lf/2+ax*h/2+1.2*ay*lf*h/Lw+ax*ay*h^2/g/Lw)*m/(lr+lf);
Fz = [Fzfl Fzfr Fzrl Fzrr];
Fz_delta = Fz - Fz_nor;

% Lateral Forces
alpha_f = delta - beta - r*lf/v;
alpha_r = -beta + r*lr/v;
alpha = [alpha_f alpha_f alpha_r alpha_r];

% % Pacejka Tire Model
% D = mu;
% C = 1.3;
% B =([9 9 14 14]+abs(Fz_delta)*0.002)./C./D;
% xm = 10*pi/180;
% E = (B.*xm-tan(pi./(2.*C)))./(B.*xm-atan(B.*xm));
% Fy = Fz*D.*sin(C.*atan(B.*alpha - E.*(B.*alpha - atan(B.*alpha))));

% Dugoff Model
[Fx, Fy, lammda] = DugoffModel(Fz, alpha, s', mu);


% alpha = (0:0.001:20*pi/180);
% D = mu*Fz;
% C = 2;
% B = 1305*180/pi./C./D;
% xm = 11*pi/180;
% E = (B.*xm-tan(pi./(2.*C)))./(B.*xm-atan(B.*xm));
% Fy = D.*sin(C.*atan(B.*alpha - E.*(B.*alpha - atan(B.*alpha))));

% NonLinear state space
z = zeros(size(x,1),1);
z(1) = (sum(Fy))/(v*m) - r; % Beta_dot [rad/s]
z(2) = ((Fy(1)+Fy(2))*lf - (Fy(3)+Fy(4))*lr)/Iz; % YawRate_dot [rad/s^2]
z(3) = r;
z(4) = v*cos((beta + yaw));
z(5) = v*sin((beta + yaw));