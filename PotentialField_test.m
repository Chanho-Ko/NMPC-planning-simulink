%% Potential Field for Obstacle

close all; clear all; clc;

% Distance based obstacle
Vx = 16.67; mu = 0.9; g = 9.81;
d = Vx^2/mu/g;
% Obstacle
A  = 10; % maximum value
a_r = 444; a_f = 4.5*sqrt(2)/2; b = 2.65*sqrt(2)/2;
L = 200; dL = 0.5;
[x,y] = meshgrid(-L:dL:L+30,-4:0.01:4);
% P_obs_r = A*exp(-((x - 100 ).^2/a_r+(y).^2/b).^2);
% P_obs = A*exp(-((x - 100 ).^2/a_f+(y).^2/b).^2);
% P_obs(:,1:(L + 104)/dL) = P_obs_r(:,1:(L + 104)/dL);
% 
P_obs_r = A*exp(-((x-60).^2/a_r+(y).^2/b).^2);
P_obs2 = A*exp(-((x-60).^2/a_f+(y).^2/b).^2);
P_obs2(:,1:(L + 60)/dL) = P_obs_r(:,1:(L + 60)/dL);
d = 20;
P_obs3 = exp(1./(((((x - 70).^2./d^2+(y-3).^2./2^2))).^2));
P_obs3f = exp(1./(((((x - 70).^2./a_f^2+(y-3).^2./2^2))).^2));
P_obs3(:,(L+100)/dL:end) = P_obs3f(:,(L+100)/dL:end);

K = ((((x - 100).^2./d^2+(y+2).^2./b^2))).^0.5 -1;
Kf = ((((x - 100).^2./a_f+(y+2).^2./b^2))).^0.5 -1;
% K(:,(L+100)/0.1:end)=Kf(:,(L+100)/0.1:end);
U = exp(-0.1*K)./(max(K,1e-2));
Uf = exp(-0.1*Kf)./(max(Kf,1e-2));
U(:,(L + 100)/dL:end) = Uf(:,(L + 100)/dL:end);

Kc = ((((x - 70).^2./d^2+(y-3).^2./1^2))).^0.5 -1;
Kcf = ((((x - 70).^2./a_f+(y-3).^2./1^2))).^0.5 -1;
Uc = 20*exp(-2*(Kc+1).^4);
Ucf = 20*exp(-2*(Kcf+1).^4);
Uc(:,(L + 70)/dL:end) = Ucf(:,(L + 70)/dL:end);

% Road edge
B = 20;
% P_road = 1/2*B*(1./(y-4)).^2 + 1/2*B*(1./(y+4)).^3; 
P_road = 1/2*B*((1./(y-4)) + (1./(y+4))).^2;
Lane = 10*exp(-(y).^2/1.1^2);

P =  1*(P_road +Lane);
figure(); grid on;

    surf(x,y,0.25*U+P, 'EdgeColor','interp'); colorbar;
map = [linspace(0,0,10)' linspace(0,0,10)' linspace(0.5,1,10)';
    [linspace(0,0,10)' linspace(0,1,10)' linspace(1,1,10)'];
    [linspace(0,0,10)' linspace(1,1,10)' linspace(1,0,10)'];
    [linspace(0,1,10)' linspace(1,1,10)' linspace(0,0,10)'];
    [linspace(1,1,10)' linspace(1,0,10)' linspace(0,0,10)']];
xlabel('x_g_l_o [m]'); ylabel('y_g_l_o [m]'); zlabel('U_e_n_v_i_r_o_n');
colormap(map)
axis([40 120 -4 4 0 50 0 30]);
% xlim([75 105])
%% Stability risk
close all;
m = 1650 + 180; % Sprung mass and unsprung mass
g = 9.81;
Iz = 3234;
Iw = 0.9; % wheel inertia
Lw = 1.6; % Track width
lf = 1.4;
lr = 1.65;
mu = 0.3;
reff = 0.353;
v = 16.67;

Tmax = 4000; amax = 0.3;
[T,alpha] = meshgrid(-Tmax:5:Tmax,-0.5:0.01:0.5);

Fz = (g*lf/2)*m/(lr+lf);
Cs = 63400; Ca = 64326;
s = 0.1;
lammda = mu*Fz.*(1+s)./(2*((Cs.*s).^2+(Ca.*tan(alpha)).^2).^0.5);
f = 1;
n = length(lammda);
for i = 1:1:n
    if lammda(i) < 1
        f = (2-lammda(i))*lammda(i);
    end
end
Fy = Ca.*tan(alpha)./(1+s).*f;

K_inst = 1 - sqrt((T/(mu*m*g*reff)).^2 + Fy.^2./Fz.^2/(mu*0.9)^2);
% J_stability = sum(sum( 10*exp(-20*K_inst)./(max(K_inst,1e-2))));
U = exp(-4*K_inst)./(max(K_inst,1e-3));


% K = +((T/(mu*m*g*reff)).^2 + (v*r/(mu*g)+sign(r)*0).^2).^0.5-1;
% U = 10*exp(-2*K)./(max(K,1e-3));
% T = -Tmax:5:Tmax;
% r = -rmax:0.01:rmax;
% U = zeros(length(T),length(r));
% for i = 1:1:length(T)
%     for j = 1:1:length(r)
%         K = -((T(i)/(mu*m*g*reff)).^2 + (v*r(j)/(mu*g)+sign(r(j))*0.15).^2).^0.5+1;
%     if K>=0
%         U(i,j) = 50*(exp(-7*(K).^2));
% %         U(i,j) = 100*(exp(0.5*(K-1).^4)-1);
%     else
%         U(i,j) = 50*(exp(20*(K).^2));
%     end
%     end
% end
% [T,a] = meshgrid(-Tmax:5:Tmax,-rmax:0.01:rmax);
surf(T,alpha,U, 'EdgeColor','interp'); colorbar; hold on;
surf(T,alpha,K_inst-0.2); hold on;

map = [linspace(0,0,10)' linspace(0,0,10)' linspace(0.5,1,10)';
    [linspace(0,0,10)' linspace(0,1,10)' linspace(1,1,10)'];
    [linspace(0,0,10)' linspace(1,1,10)' linspace(1,0,10)'];
    [linspace(0,1,10)' linspace(1,1,10)' linspace(0,0,10)'];
    [linspace(1,1,10)' linspace(1,0,10)' linspace(0,0,10)']];
xlabel('T [Nm]'); ylabel('yaw rate [rad/s]'); zlabel('U_i_n_s_t');
colormap(map)
axis([-Tmax Tmax -amax amax 0 500 0 500]);
% plot(r,exp(v.*abs(r)-0.85*0.35*9.81).^3, 'Color','b','LineWidth',3)

%%
clc; close all; clear;
[R_inst,t] = meshgrid(-0.2:0.01:1,0:0.01:10);

U = 1*exp(-4*R_inst)./(max(R_inst,1e-2));
surf(R_inst,t,U, 'EdgeColor','interp');
map = [linspace(0,0,10)' linspace(0,0,10)' linspace(0.5,1,10)';
    [linspace(0,0,10)' linspace(0,1,10)' linspace(1,1,10)'];
    [linspace(0,0,10)' linspace(1,1,10)' linspace(1,0,10)'];
    [linspace(0,1,10)' linspace(1,1,10)' linspace(0,0,10)'];
    [linspace(1,1,10)' linspace(1,0,10)' linspace(0,0,10)']];
xlabel('R_{inst}'); ylabel('Time [sec]'); zlabel('U_i_n_s_t');
colormap(map)