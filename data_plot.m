clc; clear;
%% Scenario 1 - Known obstacle with 60kph and 80kph
close all
% Load Data
load('60kph_high_obs70m_');
load('states3');
data.input = inputs.Data;
data.state = states.Data;
load('inputs4');
load('states4');
data80kph.input = inputs.Data;
data80kph.state = states.Data;

Ts = 0.1;
t = (0:Ts:15); n = length(t);

% Trajectories
figure(); hold on;
plot(data.state(:,4),data.state(:,5),'Color','b','LineWidth',3)
plot(data80kph.state(:,4),data80kph.state(:,5),'Color','r','LineWidth',3)
patch([100 104.5 104.5 100], [-3 -3 -1 -1],'m') % obstacle
patch([130 134.5 134.5 130], [1 1 3 3],'m') 

x_final =  230;
axis([0 x_final -4 4])
plot(linspace(0,x_final,200),0*linspace(0,x_final,200),'--','Color','k','LineWidth',6)
plot(linspace(0,x_final,200),0*linspace(0,x_final,200)-4,'-','Color','k','LineWidth',6)
plot(linspace(0,x_final,200),0*linspace(0,x_final,200)+4,'-','Color','k','LineWidth',6)
legend('60 km/h','80 km/h','Obstacles')
xlabel('x_g_l_o [m]'); ylabel('y_g_l_o [m]');

% steering angle
figure(); hold on; 
plot(t,data.input(:,1),'Color','b','LineWidth',2)
plot(t,data80kph.input(:,1),'Color','r','LineWidth',2)
legend('60 km/h','80 km/h')
xlabel('Time [sec]'); ylabel('Steering Angle [deg]');

% torque input
figure(); hold on;
plot(t,data.input(:,2),'Color','b','LineWidth',2)
plot(t,data80kph.input(:,2),'Color','r','LineWidth',2)
legend('60 km/h','80 km/h')
xlabel('Time [sec]'); ylabel('Torque Input [Nm]');

% speed
figure(); hold on;
plot((0:0.001:15),data.state(:,6),'Color','b','LineWidth',2)
plot((0:0.001:15),data80kph.state(:,6),'Color','r','LineWidth',2)
legend('60 km/h','80 km/h')
xlabel('Time [sec]'); ylabel('Longitudinal Speed [km/h]');


% Yaw rate
r_boundary = 0.85*0.3*9.81./(data.state(:,6)./3.6)*180/pi; 
figure(); hold on;
plot((0:0.001:15),data.state(:,2),'Color','b','LineWidth',2); hold on
plot((0:0.001:15),r_boundary,'Color','r','LineWidth',2); 
plot((0:0.001:15),-r_boundary,'Color','r','LineWidth',2); 
xlabel('Time [sec]'); ylabel('Yaw rate [deg/s]');
legend('Yaw rate', 'Boundary')
axis([0 15 -15 15])

%% Scenario 2 - Popup Obstacle with 60kph
clear; close all
s = -2*linspace(1,1,50001);
mu = 0.5;

% Load Data
load('60kph_05_pu_125_wo_states_0906');
load('60kph_05_pu_125_wo_inputs_0906');

data.input = inputs.Data;
data.state = states.Data;
Ts = 0.05;
t_final = 5.1;
t = (0:Ts:t_final); n = length(t);

% Trajectories
x_obs = 50;
figure(); hold on; grid on;
plot(data.state(:,4),data.state(:,5),'Color','b','LineWidth',3)
plot((0:0.01:500),s,'--r','LineWidth',3)
plot(50-20.83,-2,'x','MarkerSize',13,'LineWidth',3)
patch([x_obs x_obs+4.5 x_obs+4.5 x_obs], [-3 -3 -1 -1],'m') % obstacle
% patch([130 134.5 134.5 130], [1 1 3 3],'m') 

x_final =  200;
axis([0 x_final -4 4])
plot(linspace(0,x_final,200),0*linspace(0,x_final,200),'--','Color','k','LineWidth',6)
plot(linspace(0,x_final,200),0*linspace(0,x_final,200)-4,'-','Color','k','LineWidth',6)
plot(linspace(0,x_final,200),0*linspace(0,x_final,200)+4,'-','Color','k','LineWidth',6)
legend('Vehicle Path','Reference','Popup Obstacle Detected','Obstacle')
xlabel('x_g_l_o [m]'); ylabel('y_g_l_o [m]');

% steering angle
figure(); hold on;  grid on;
plot(t,(data.input(:,1))*180/pi,'Color','b','LineWidth',2)
xlabel('Time [sec]'); ylabel('Steering Angle [deg]');
% axis([0 7.5 -15 10])
xlim([0 5])

% torque input
figure(); hold on; grid on;
plot(t,data.input(:,2),'Color','b','LineWidth',2)
xlabel('Time [sec]'); ylabel('Torque Input [Nm]');

% speed
figure(); hold on; grid on;
plot((0:0.001:t_final),data.state(:,6),'Color','b','LineWidth',2)
xlabel('Time [sec]'); ylabel('Longitudinal Speed [km/h]');

% side slip
figure(); hold on; grid on;
plot((0:0.001:t_final),data.state(:,1),'Color','b','LineWidth',2)
xlabel('Time [sec]'); ylabel('Side Slip Angle [deg]');

% Yaw rate
r_boundary = 0.85*mu*9.81./(data.state(:,6)./3.6)*180/pi; 
figure(); hold on; grid on;
plot((0:0.001:t_final),data.state(:,2),'Color','b','LineWidth',2); hold on
plot((0:0.001:t_final),r_boundary,'Color','r','LineWidth',2); 
plot((0:0.001:t_final),-r_boundary,'Color','r','LineWidth',2); 
xlabel('Time [sec]'); ylabel('Yaw rate [deg/s]');
legend('Yaw rate', 'Boundary')
axis([0 4.5 -25 25])
% xlim([0 10])
% axis([0 5.9 -40 45])
%% TEST
% Load Data
load('inputs_temp2');
load('states_temp2');
data.input = inputs.Data;
data.state = states.Data;


Ts = 0.05;
t = (0:Ts:15); n = length(t);

% Trajectories tracking
figure(); hold on;
plot(data.state(:,4),data.state(:,5),'Color','b','LineWidth',3)
plot((0:0.01:300),s,'--r','LineWidth',3)
% patch([100 104.5 104.5 100], [-3 -3 -1 -1],'m') % obstacle
% patch([130 134.5 134.5 130], [1 1 3 3],'m') 

x_final =  230;
axis([0 x_final -4 4])
plot(linspace(0,x_final,200),0*linspace(0,x_final,200),'--','Color','k','LineWidth',6)
plot(linspace(0,x_final,200),0*linspace(0,x_final,200)-4,'-','Color','k','LineWidth',6)
plot(linspace(0,x_final,200),0*linspace(0,x_final,200)+4,'-','Color','k','LineWidth',6)
legend('vehicle path','reference')
xlabel('x_g_l_o [m]'); ylabel('y_g_l_o [m]');

% Yaw tracking
figure(); hold on;
plot(data.state(:,4),(data.state(:,3)).*pi/180,'Color','b','LineWidth',3)
s_next = [s(2:end) 0];
yaw = atan2(s_next - s, 0.01); yaw(end) = yaw(end-1);
plot(linspace(0,300,length(yaw)),yaw,'--r','LineWidth',3)
axis([0 x_final -0.3 0.3])

%% Friction Ellipse
close all
% Load Data
load('60kph_05_pu_125_wo_states_0906');
load('60kph_05_pu_125_wo_inputs_0906');
data.input = inputs.Data;
data.state = states.Data;

Ts = 0.05;
t_final = 5.1;
t = (0:Ts:t_final);

m = 1650 + 180; % Sprung mass and unsprung mass
g = 9.81;
Iz = 3234;
Iw = 0.9; % wheel inertia
Lw = 1.6; % Track width
lf = 1.4;
lr = 1.65;
mu = 0.5;
reff = 0.353;

% w_dot = data.wdot;
% w_dot = decimate(w_dot,50);
T = data.input(:,2);
v = data.state(:,6);
v = decimate(v,50)/3.6;
r = data.state(:,2);
r = decimate(r,50)*pi/180;
K = 1- sqrt(((T-4*Iw*0)/(reff*mu*m*g)).^2+(v.*r/(mu*g)+sign(r)*0.15).^2);

plot(t, K,'Color','b','LineWidth',2); hold on; grid on;
plot(t,0*t+1,'--r','LineWidth',2);
plot(t,0*t,'--r','LineWidth',2);
% xlim([0 5])
xlabel('Time [sec]'); ylabel('K_i_n_s_t');

% v = 16.667;
% plot((-0.3:0.01:0.3),mu*m*g*reff*(1 - (v*(-0.3:0.01:0.3)/(mu*g)+0.15).^2).^0.5); hold on
% plot((-0.3:0.01:0.3),-mu*m*g*reff*(1 - (v*(-0.3:0.01:0.3)/(mu*g)+0.15).^2).^0.5)
% plot(r2*pi/180,T)

%% Moving obstacle
close all; clc;
addpath('simulation_final');
% Load Data
load('80kph_09_mv70_inputs.mat');
load('80kph_09_mv70_states_0824.mat');
% load('60kph_05_pu_175_wo_states_0906(2)');
% load('60kph_05_pu_175_wo_inputs_0906(2)');
% data_com.input = inputs.Data;
% data_com.state = states.Data;

% load('80kph_09_mv50_inputs_0824.mat');
% load('80kph_09_mv50_states_08234.mat');
% % load('60kph_05_pu_175_wo_states_0906(2)');
% % load('60kph_05_pu_175_wo_inputs_0906(2)');
data.input = inputs.Data;
data.state = states.Data;

Ts = 0.05;
t_final = (length(data.input))*Ts;
t = (0.001:Ts:t_final); n = length(t);

% Trajectories
figure(); hold on;grid on;
plot(data.state(:,4),data.state(:,5),'Color','b','LineWidth',3)
plot((0:0.01:500),-2*ones(length((0:0.01:500)),1),'--r','LineWidth',3)
patch([111.7 111.7+4.5 111.7+4.5 111.7], [-3 -3 -1 -1],'m') % obstacle
patch([209 209+4.5 209+4.5 209], [-3 -3 -1 -1],'m') 

x_final =  400;
axis([0 x_final -4 4])
plot(linspace(0,x_final,200),0*linspace(0,x_final,200),'--','Color','k','LineWidth',6)
plot(linspace(0,x_final,200),0*linspace(0,x_final,200)-4,'-','Color','k','LineWidth',6)
plot(linspace(0,x_final,200),0*linspace(0,x_final,200)+4,'-','Color','k','LineWidth',6)
legend('Vehicle Path','Reference','Moving Obstacle')
xlabel('x_g_l_o [m]'); ylabel('y_g_l_o [m]');

x_lim = 25;
% steering angle
subplot(2,2,1); hold on;
plot(t,(data.input(:,1))*180/pi,'Color','b','LineWidth',1)
% plot(t,(data_com.input(:,1))*180/pi,'-','Color','k','LineWidth',1)
xlabel('Time [sec]'); ylabel('Steering Angle [deg]');
% legend('w/ Priority factor', 'w/o Priority factor')
axis([0 x_lim -1 1])
% xlim([0 x_lim])


% torque input
subplot(2,2,2); hold on;
plot(t,data.input(:,2),'Color','b','LineWidth',1)
% plot(t,data_com.input(:,2),'Color','k','LineWidth',1)
xlabel('Time [sec]'); ylabel('Torque Input [Nm]');
% legend('w/ Priority factor', 'w/o Priority factor')
xlim([0 x_lim])

t_state = (0.001:0.001:25);
% yaw rate
subplot(2,2,3); hold on;
plot(t_state,data.state(int16(t_state/0.001),2),'Color','b','LineWidth',1)
% plot((0:0.001:t_final),data_com.state(:,2),'Color','k','LineWidth',1)

xlabel('Time [sec]'); ylabel('Yaw rate [deg/s]');
% legend('w/ Priority factor', 'w/o Priority factor')
axis([0 x_lim -1 1])
% xlim([0 x_lim])

% speed
subplot(2,2,4); hold on;
plot(t_state,data.state(int16(t_state/0.001),6)/3.6,'Color','b','LineWidth',1)
% plot((0:0.001:t_final),data_com.state(:,6)/3.6,'Color','k','LineWidth',1)

plot(t,70/3.6*ones(length(t),1),'--k','LineWidth',1)
xlabel('Time [sec]'); ylabel('Longitudinal Speed [m/s]');
plot(t,80/3.6*ones(length(t),1),'--r','LineWidth',1)
legend('Vehicle', 'Obstacle', 'Reference')
axis([0 x_lim 69/3.6 81/3.6])

%% Instability risk assessment
clc; clear; close all;
addpath('simulation_final');


m = 1650 + 180; % Sprung mass and unsprung mass
g = 9.81;
Iz = 3234;
Iw = 0.9; % wheel inertia
Lw = 1.6; % Track width
lf = 1.4;
lr = 1.65;
reff = 0.353;
h = 0.53;

data_IRA = load('1202_curve_IRA.mat');
data_inde = load('1207_curve_STenv+inde.mat');
data_off = load('60kph_straight_pu_off.mat');

% [beta(deg) r(deg/s) psi(deg) x(m) y(m) v(kph) ax(g) ay(g) wdot s_fl s_fr s_rl s_rr mu]
arrays = data_IRA.data.Data;
x_IRA = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];
u_IRA = arrays(:,19:20);
u_IRA(:,1) = (arrays(:,25)+arrays(:,26))/2;

ax = arrays(:,7)*9.81;
ay = arrays(:,8)*9.81;
s = arrays(:,10:13);
mu = arrays(1,14);
% u_IRA(:,2) = (arrays(:,21)+arrays(:,22)+arrays(:,23)+arrays(:,24));
arrays = data_inde.data.Data;
x_inde = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];
u_inde = arrays(:,19:20);
u_inde(:,1) = (arrays(:,25)+arrays(:,26))/2;
% u_inde(:,2) = (arrays(:,21)+arrays(:,22)+arrays(:,23)+arrays(:,24));
arrays = data_off.data.Data;
x_off = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];
u_off = arrays(:,19:20);
u_off(:,1) = (arrays(:,25)+arrays(:,26))/2;
% u_off(:,2) = (arrays(:,21)+arrays(:,22)+arrays(:,23)+arrays(:,24));

Ts = 0.001;
t_final = 17;
t = (0.001:Ts:t_final); n = length(t);



% S = sqrt(x(:,4).^2 + x(:,5).^2);
v_ref = load('v_profile_cur50_0.3_v-s.mat');
v_ref = v_ref.v;
y_ref = load('curve_left_R50_y-s.mat');
x_ref = load('curve_left_R50_x-s.mat');
y_ref = y_ref.y;
x_ref = x_ref.x;




% % Trajectories tracking
% figure(); hold on;
% plot(x(:,4),x(:,5),'Color','b','LineWidth',3)
% plot(x_ref(int16(S/0.01)),y_ref(int16(S/0.01)),'--r','LineWidth',3)
% axis([0 110 -10 150])
% legend('Vehicle path','Reference')

x_lim = 17;
figure(1);
% steering angle
subplot(2,2,1); hold on;
% plot(t,u_IRA(int16(t/Ts),1)*180/pi,'Color','b','LineWidth',1)
% plot(t,u_off(int16(t/Ts),1)*180/pi,'Color','r','LineWidth',1)
% plot(t,u_inde(int16(t/Ts),1)*180/pi,'Color','k','LineWidth',1)
plot(t,u_IRA(int16(t/Ts),1),'Color','b','LineWidth',1)
% plot(t,u_off(int16(t/Ts),1),'-k','LineWidth',1)
plot(t,u_inde(int16(t/Ts),1),'-r','LineWidth',1)
% plot(t,u(:,1)*180/pi,'Color','b','LineWidth',1)
xlabel('Time [sec]'); ylabel('Steering Angle [deg]','FontSize', 14);
legend('IRPF','SCs') 
xlim([0 x_lim])

% torque input
subplot(2,2,2); hold on;
% plot(t,u(:,2),'Color','b','LineWidth',1)
plot(t,u_IRA(int16(t/Ts),2),'Color','b','LineWidth',1)
% plot(t,u_off(int16(t/Ts),2),'-k','LineWidth',1)

u_inde(1:100,2) = 0;
plot(t,u_inde(int16(t/Ts),2),'-r','LineWidth',1)
axis([2 7 -2000 3000])
% plot(t,-(m+4*Iw/reff^2)*mu*g*reff*sqrt(1-(ay/(mu*0.9*g).^2))+0.0225*m*g*reff,'-r','LineWidth',1)
% plot(t,(m+4*Iw/reff^2)*mu*g*reff*sqrt(1-(ay/(mu*0.9*g).^2))+0.0225*m*g*reff,'-r','LineWidth',1)
xlabel('Time [sec]'); ylabel('Torque Input [Nm]','FontSize', 14);
% legend('Torque input', 'Torque input bound')
legend('IRPF','SCs') 
xlim([0 x_lim])

% yaw rate
subplot(2,2,3); hold on; 
% plot(t,x(:,2)*180/pi,'Color','b','LineWidth',1)

plot(t,x_IRA(int16(t/Ts),2)*180/pi,'Color','b','LineWidth',1)
% plot(t,x_off(int16(t/Ts),2)*180/pi,'Color','k','LineWidth',1)
plot(t,x_inde(int16(t/Ts),2)*180/pi,'Color','r','LineWidth',1)

plot(t,0.85*g*mu./x_inde(int16(t/Ts),6)*180/pi,':r','LineWidth',1)
plot(t,-0.85*g*mu./x_inde(int16(t/Ts),6)*180/pi,':r','LineWidth',1)
legend('IRPF','SCs','bond for SCs') 
xlabel('Time [sec]'); ylabel('Yaw rate [deg/s]','FontSize', 14);
xlim([0 x_lim])


% axis([0 20 -30 35])


% speed
v_ref = load('v_profile_cur50_0.3_v-s.mat');
v_ref = v_ref.v;
subplot(2,2,4); hold on; 
% plot(t,x(:,6),'Color','b','LineWidth',1)   
% plot(t,v_ref(int16(S/0.01)),'--r','LineWidth',1)

plot(t,x_IRA(int16(t/Ts),6),'Color','b','LineWidth',1)
% plot(t,x_off(int16(t/Ts),6),'Color','k','LineWidth',1)
plot(t,x_inde(int16(t/Ts),6),'Color','r','LineWidth',1)

S = sqrt(x_IRA(int16(t/Ts),4).^2+x_IRA(int16(t/Ts),5).^2);
plot(t,v_ref(int16(S/0.01)),'--r','LineWidth',1)
legend('IRPF','SCs','Reference') 
xlabel('Time [sec]'); ylabel('Longitudinal Speed [m/s]','FontSize', 14);
xlim([0 x_lim])

Fz_vec =[];
Fx_vec =[];
Fy_vec =[];

x = x_IRA;
u = u_IRA;
for i = 1:1:length(arrays)
    [z, Fz, Fx, Fy ,lammda] = nonlinVehicleModel_val(x(i,:),s(i,:)',u(i,:),ax(i),ay(i),mu);
    Fz_vec = [Fz_vec; Fz];
    Fx_vec = [Fx_vec; Fx];
    Fy_vec = [Fy_vec; Fy];
end

% figure(); hold on; grid on;
% plot(Fx_vec(:,1));
% plot(Fx_vec(:,2));
% plot(Fx_vec(:,3));
% plot(Fx_vec(:,4));

K_fl = 1 - sqrt(Fx_vec(:,1).^2./Fz_vec(:,1).^2/mu^2 + Fy_vec(:,1).^2./Fz_vec(:,1).^2/(mu*0.9)^2);
K_fr = 1 - sqrt(Fx_vec(:,2).^2./Fz_vec(:,2).^2/mu^2 + Fy_vec(:,2).^2./Fz_vec(:,2).^2/(mu*0.9)^2);
K_rl = 1 - sqrt(Fx_vec(:,3).^2./Fz_vec(:,3).^2/mu^2 + Fy_vec(:,3).^2./Fz_vec(:,3).^2/(mu*0.9)^2);
K_rr = 1 - sqrt(Fx_vec(:,4).^2./Fz_vec(:,4).^2/mu^2 + Fy_vec(:,4).^2./Fz_vec(:,4).^2/(mu*0.9)^2);
K_body = 1- sqrt((ax/(mu*g)).^2 + (ay/(mu*g*0.9)).^2);
index = zeros(n,1);
for i = 1:1:n
    [min_val, index(i)] = min([K_fl(i) K_fr(i) K_rl(i) K_rr(i) K_body(i)]);
end
mu = mu;
% K_fl = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2  + Fy_vec(:,1).^2./Fz_vec(:,1).^2/(mu*0.9)^2);
% K_fr = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,2).^2./Fz_vec(:,2).^2/(mu*0.9)^2);
% K_rl = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,3).^2./Fz_vec(:,3).^2/(mu*0.9)^2);
% K_rr = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,4).^2./Fz_vec(:,4).^2/(mu*0.9)^2);
% K_body = 1 - sqrt(((u(:,2)-0.0225*m*g*reff)/(mu*(m+4*Iw/reff^2)*g*reff)).^2 + (0.85*(x(:,6).*x(:,2)+beta_dot)./(mu*g*0.9)).^2);
% K_body =  1- sqrt((ax/(mu*g)).^2+(ay/(mu*g*0.9)).^2);
figure(2);
subplot(2,1,1); hold on;
plot(t,K_fl(int16(t/Ts)));
plot(t,K_fr(int16(t/Ts)));
plot(t,K_rl(int16(t/Ts)));
plot(t,K_rr(int16(t/Ts)));
plot(t,K_body(int16(t/Ts)));
plot(t,zeros(length(t),1),'-r');
legend('FL','FR','RL','RR','Body','Bound')
xlabel('Time [sec]'); ylabel('R_{inst}');
axis([0 x_lim -0.6 1])
subplot(2,1,2); hold on; grid on;
plot(t,index)
axis([0 x_lim 0.5 5.5])
xlabel('Time [sec]'); ylabel('i_{target}')
% 
% figure(3);
% subplot(2,1,1); hold on;
% t = 10:0.001:12;
% plot(t,u(int16(t/Ts),1)*180/pi);
% subplot(2,1,2);
% plot(t,index(int16(t/Ts)));

% % side slip
% figure(); hold on; 
% plot(t,x(:,1)*180/pi,'Color','b','LineWidth',1)
% plot(t,atan(0.02*mu*g)*180/pi,'-r','LineWidth',1)
% plot(t,-atan(0.02*mu*g)*180/pi,'-r','LineWidth',1)
% legend('Yaw rate', 'Yaw rate bound')
% xlabel('Time [sec]'); ylabel('Yaw rate [deg/s]');
% xlim([0 x_lim])


%% Comparison set
clc; clear; close all;

m = 1650 + 180; % Sprung mass and unsprung mass
g = 9.81;
Iz = 3234;
Iw = 0.9; % wheel inertia
Lw = 1.6; % Track width
lf = 1.4;
lr = 1.65;
reff = 0.353;
h = 0.53;

data = load('1123_60kph_0.3_pu_IRA(gaintune).mat');

% [beta(deg) r(deg/s) psi(deg) x(m) y(m) v(kph) ax(g) ay(g) wdot s_fl s_fr s_rl s_rr mu]
arrays = data.data.Data;
Ts = 0.001;
t_final = length(arrays)*0.001;
t = (0:Ts:t_final-Ts); n = length(t);

x = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];
ax = arrays(:,7)*9.81;
ay = arrays(:,8)*9.81;
s = arrays(:,10:13);
mu = arrays(1,14);
u = arrays(:,19:20);

% plot(x(:,4), x(:,5))

xp = diff(x(:,4))/Ts;
xpp = diff(xp)/Ts;
yp = diff(x(:,5))/Ts;
ypp = diff(yp)/Ts;
% plot(yp)
k = abs(xp(1:end-1).*ypp-yp(1:end-1).*xpp)/(xp(1:end-1).^2+yp(1:end-1).^2).^1.5;
plot(k)

%% curve

close all
% Load Data
yref = load('curve_right_left_R100.mat');
data = load('1127_60kph_0.3_curve_IRA.mat');
Ts = 0.001;
t_final = 15;
t = (0:Ts:t_final); n = length(t);

% [beta(deg) r(deg/s) psi(deg) x(m) y(m) v(kph) ax(g) ay(g) wdot s_fl s_fr s_rl s_rr mu]
arrays = data.data.Data;

x = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];
ax = arrays(:,7)*9.81;
ay = arrays(:,8)*9.81;
s = arrays(:,10:13);
mu = arrays(1,14);
u = arrays(:,19:20);

% Trajectories
figure(); hold on;
% plot(data.state(:,4),data.state(:,5),'Color','b','LineWidth',3)
% plot(data80kph.state(:,4),data80kph.state(:,5),'Color','r','LineWidth',3)
% patch([100 104.5 104.5 100], [-3 -3 -1 -1],'m') % obstacle
% patch([130 134.5 134.5 130], [1 1 3 3],'m') 
xref = (0:0.01:300);
plot(xref,yref.y)
plot(x(:,4),x(:,5))

x_final =  230;
axis([0 x_final -4 4])
plot(linspace(0,x_final,200),0*linspace(0,x_final,200),'--','Color','k','LineWidth',6)
plot(linspace(0,x_final,200),0*linspace(0,x_final,200)-4,'-','Color','k','LineWidth',6)
plot(linspace(0,x_final,200),0*linspace(0,x_final,200)+4,'-','Color','k','LineWidth',6)
legend('60 km/h','80 km/h','Obstacles')
xlabel('x_g_l_o [m]'); ylabel('y_g_l_o [m]');

%% Animation
close all; clear; clc;
addpath('simulation_final');

data = load('1202_curve_IRA.mat');
arrays = data.data.Data;
x = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];
data = load('1207_curve_STenv+inde.mat');
arrays = data.data.Data;
x_inde = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];
data = load('1212_curve_IRA.mat');
arrays = data.data.Data;
x_off = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];

Ts = 0.001;
t_final = length(arrays)*0.001;
t = (0:Ts:t_final-Ts); n = length(t);

S = sqrt(x(:,4).^2 + x(:,5).^2);
y_ref = load('curve_left_R50_y-s.mat');
x_ref = load('curve_left_R50_x-s.mat');
y_ref = y_ref.y;
x_ref = x_ref.x;

b_r = load('curve_boundary_right.mat');
b_l = load('curve_boundary_left.mat');
lane = load('curve_lane.mat');


h1 = figure('Color', 'White');
h2 = figure('Color', 'White');

% axis tight manual % this ensures that getframe() returns a consistent size
% Trajectories trackin


filename = 'curve_03_IRA.gif'; %GIF file is automatically created

% Vehicle demension
a = 2;
b = 0.9;
r = sqrt(a^2+b^2);
theta = atan2(b,a);

t = int16((0.001:0.001:20)/0.001);
t_inde = int16((0.001:0.001:13)/0.001);

subplot(1,3,1:2)
p1 = plot(x(t,4),x(t,5),'Color','b','LineWidth',2); hold on;
p2 = plot(x_ref,y_ref,'--r','LineWidth',2);
p3 = plot(b_r.x,b_r.y,'-','Color','k','LineWidth',1);
p4 = plot(lane.x,lane.y,'--','Color','k','LineWidth',1);
p5 = plot(b_l.x,b_l.y,'-','Color','k','LineWidth',1);
p6 = patch([99.2 101 101 99.2], [50 50 54 54],'m'); % obstacle
p7 = plot(x_inde(t_inde,4),x_inde(t_inde,5),'Color','r','LineWidth',2);
% p8 = plot(x_off(t,4),x(t,5),'k:','LineWidth',2);

subplot(1,3,3)
plot(x(t,4),x(t,5),'Color','b','LineWidth',3); hold on;
plot(x_ref,y_ref,'--r','LineWidth',1)
plot(b_r.x,b_r.y,'-','Color','k','LineWidth',1)
plot(lane.x,lane.y,'--','Color','k','LineWidth',1)
plot(b_l.x,b_l.y,'-','Color','k','LineWidth',1)
plot(x_inde(t_inde,4),x_inde(t_inde,5),'Color','r','LineWidth',2);

patch([99 101 101 99], [50 50 54 54],'m') % obstacle
% for t = [0.001]
for t = 0.001:0.1:t_final
    x_glo = x(int16(t/Ts),4);
    y_glo = x(int16(t/Ts),5);
    yaw = x(int16(t/Ts),3);
    x_ver = [x_glo-r*cos(yaw+theta) x_glo+r*cos(yaw-theta) x_glo+r*cos(yaw+theta) x_glo-r*cos(yaw-theta)];
    y_ver = [y_glo-r*sin(yaw+theta) y_glo+r*sin(yaw-theta) y_glo+r*sin(yaw+theta) y_glo-r*sin(yaw-theta)];
    

    subplot(1,3,1:2)
%     set(h1,'Position',[10 10 500 500])
%     set(h2,'Position',[10 10 500 500])
    
    view(2),axis equal
    axis([30 110 -10 90])
    xlabel('x_{glo} [m]'); ylabel('y_{glo} [m]');
    if t > 0.001
        delete(h_patch1)
    end
    h_patch1 = patch(x_ver, y_ver, 'c');
    legend([p1 p7 p2 p3 p4 p6 h_patch1],{'IRPF','SCs','Reference','Road bound','Lane','Obstacle','Vehicle'},'FontSize',12)
    frame1 = getframe(h1);
    
    
    subplot(1,3,3)
    view(2), axis equal
    axis([90 105 35 70])
    if t == 0.001
        pause(10)
    end
    if t>0.001
        delete(h_patch2)
    end
    h_patch2 = patch(x_ver, y_ver, 'c');
    
    
    
    %Create animated GIF file
    
    frame2 = getframe(h2);
%     if t == 0.001
%         pause(15)
%     end
%     im = frame2im(frame); 
%     [imind,cm] = rgb2ind(im,256); 
%     % Write to the GIF File 
%     if t == 0 
%         imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%     else 
%         imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%     end 
%        
%     if t<t_final
%         clf; % Clear current figure.
%     end
end    

%% Rinst RMS
close all; clear; clc;
addpath('simulation_final');
m = 1650 + 180; % Sprung mass and unsprung mass
g = 9.81;
Iz = 3234;
Iw = 0.9; % wheel inertia
Lw = 1.6; % Track width
lf = 1.4;
lr = 1.65;
reff = 0.353;
h = 0.53;

data = load('1202_curve_IRA.mat');
arrays = data.data.Data;
Ts = 0.001;
t_final = length(arrays)*0.001;
t = (0:Ts:t_final-Ts); n = length(t);
x = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];
ax = arrays(:,7)*9.81;
ay = arrays(:,8)*9.81;
s = arrays(:,10:13);
mu = arrays(1,14);
u = arrays(:,19:20);

Fz_vec =[];
Fx_vec =[];
Fy_vec =[];

for i = 1:1:length(arrays)
    [z, Fz, Fx, Fy ,lammda] = nonlinVehicleModel_val(x(i,:),s(i,:)',u(i,:),ax(i),ay(i),mu);
    Fz_vec = [Fz_vec; Fz];
    Fx_vec = [Fx_vec; Fx];
    Fy_vec = [Fy_vec; Fy];
end

% figure(); hold on; grid on;
% plot(Fx_vec(:,1));
% plot(Fx_vec(:,2));
% plot(Fx_vec(:,3));
% plot(Fx_vec(:,4));
mu = mu*1.04;
K_fl = 1 - sqrt(Fx_vec(:,1).^2./Fz_vec(:,1).^2/mu^2 + Fy_vec(:,1).^2./Fz_vec(:,1).^2/(mu*0.9)^2);
K_fr = 1 - sqrt(Fx_vec(:,2).^2./Fz_vec(:,2).^2/mu^2 + Fy_vec(:,2).^2./Fz_vec(:,2).^2/(mu*0.9)^2);
K_rl = 1 - sqrt(Fx_vec(:,3).^2./Fz_vec(:,3).^2/mu^2 + Fy_vec(:,3).^2./Fz_vec(:,3).^2/(mu*0.9)^2);
K_rr = 1 - sqrt(Fx_vec(:,4).^2./Fz_vec(:,4).^2/mu^2 + Fy_vec(:,4).^2./Fz_vec(:,4).^2/(mu*0.9)^2);
K_body = 1- sqrt((ax/(mu*g)).^2 + (ay/(mu*g*0.9)).^2);
index = zeros(n,1);
for i = 1:1:n
    [min_val, index(i)] = min([K_fl(i) K_fr(i) K_rl(i) K_rr(i) K_body(i)]);
end

% K_fl = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2  + Fy_vec(:,1).^2./Fz_vec(:,1).^2/(mu*0.9)^2);
% K_fr = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,2).^2./Fz_vec(:,2).^2/(mu*0.9)^2);
% K_rl = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,3).^2./Fz_vec(:,3).^2/(mu*0.9)^2);
% K_rr = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,4).^2./Fz_vec(:,4).^2/(mu*0.9)^2);
% K_body = 1 - sqrt(((u(:,2)-0.0225*m*g*reff)/(mu*(m+4*Iw/reff^2)*g*reff)).^2 + (0.85*(x(:,6).*x(:,2))./(mu*g*0.9)).^2);
% K_rms = ((K_fl+K_fr+K_rl+K_rr+K_body)/5);

subplot(2,1,1);hold on;
% plot(t,K_fl,'LineWidth',1)
% plot(t,K_fr,'LineWidth',1)
% plot(t,K_rl,'LineWidth',1)
% plot(t,K_rr,'LineWidth',1)
% plot(t,K_body,'LineWidth',1)
K_min = min(K_fl, K_fr);
K_min2 = min(K_rl, K_rr);
K_min2 = min(K_min2, K_body);
K_min = min(K_min2, K_min);
plot(t,K_min,'b','LineWidth',1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data = load('60kph_straight_pu_inde.mat');
% arrays = data.data.Data;
% Ts = 0.001;
% t_final = length(arrays)*0.001;
% t = (0:Ts:t_final-Ts); n = length(t);
% x = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];
% ax = arrays(:,7)*9.81;
% ay = arrays(:,8)*9.81;
% s = arrays(:,10:13);
% mu = arrays(1,14);
% u = arrays(:,19:20);
% 
% Fz_vec =[];
% Fx_vec =[];
% Fy_vec =[];
% 
% for i = 1:1:length(arrays)
%     [z, Fz, Fx, Fy ,lammda] = nonlinVehicleModel_val(x(i,:),s(i,:)',u(i,:),ax(i),ay(i),mu);
%     Fz_vec = [Fz_vec; Fz];
%     Fx_vec = [Fx_vec; Fx];
%     Fy_vec = [Fy_vec; Fy];
% end
% 
% % figure(); hold on; grid on;
% % plot(Fx_vec(:,1));
% % plot(Fx_vec(:,2));
% % plot(Fx_vec(:,3));
% % plot(Fx_vec(:,4));
% 
% K_fl = 1 - sqrt(Fx_vec(:,1).^2./Fz_vec(:,1).^2/mu^2 + Fy_vec(:,1).^2./Fz_vec(:,1).^2/(mu*0.9)^2);
% K_fr = 1 - sqrt(Fx_vec(:,2).^2./Fz_vec(:,2).^2/mu^2 + Fy_vec(:,2).^2./Fz_vec(:,2).^2/(mu*0.9)^2);
% K_rl = 1 - sqrt(Fx_vec(:,3).^2./Fz_vec(:,3).^2/mu^2 + Fy_vec(:,3).^2./Fz_vec(:,3).^2/(mu*0.9)^2);
% K_rr = 1 - sqrt(Fx_vec(:,4).^2./Fz_vec(:,4).^2/mu^2 + Fy_vec(:,4).^2./Fz_vec(:,4).^2/(mu*0.9)^2);
% K_body = 1- sqrt((ax/(mu*g)).^2 + (ay/(mu*g*0.9)).^2);
% index = zeros(n,1);
% for i = 1:1:n
%     [min_val, index(i)] = min([K_fl(i) K_fr(i) K_rl(i) K_rr(i) K_body(i)]);
% end
% mu = mu;
% % K_fl = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2  + Fy_vec(:,1).^2./Fz_vec(:,1).^2/(mu*0.9)^2);
% % K_fr = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,2).^2./Fz_vec(:,2).^2/(mu*0.9)^2);
% % K_rl = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,3).^2./Fz_vec(:,3).^2/(mu*0.9)^2);
% % K_rr = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,4).^2./Fz_vec(:,4).^2/(mu*0.9)^2);
% % K_body = 1 - sqrt(((u(:,2)-0.0225*m*g*reff)/(mu*(m+4*Iw/reff^2)*g*reff)).^2 + (0.85*(x(:,6).*x(:,2))./(mu*g*0.9)).^2);
% % K_rms = ((K_fl+K_fr+K_rl+K_rr+K_body)/5);
% 
% % subplot(2,1,1);hold on;
% % plot(t,K_fl,'LineWidth',1)
% % plot(t,K_fr,'LineWidth',1)
% % plot(t,K_rl,'LineWidth',1)
% % plot(t,K_rr,'LineWidth',1)
% % plot(t,K_body,'LineWidth',1)
% K_min = min(K_fl, K_fr);
% K_min2 = min(K_rl, K_rr);
% K_min2 = min(K_min2, K_body);
% K_min = min(K_min2, K_min);
% plot(t,K_min,'-k','LineWidth',1)
% 
% 

plot(t,zeros(length(t),1),'r:');

% legend('Proposed','Env','Inde+Env','Bound')
% legend('IRPF','SCs')
xlabel('Time [sec]'); ylabel('R_{inst}');
axis([0 15 -0.3 1])

subplot(2,1,2); grid on;
plot(t, index)
xlabel('Time [sec]'); ylabel('i_{target}');
axis([0 15 0.5 5.5])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data = load('60kph_straight_pu_inde.mat');
% arrays = data.data.Data;
% Ts = 0.001;
% t_final = length(arrays)*0.001;
% t = (0:Ts:t_final-Ts); n = length(t);
% x = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];
% ax = arrays(:,7)*9.81;
% ay = arrays(:,8)*9.81;
% s = arrays(:,10:13);
% mu = arrays(1,14);
% u = arrays(:,19:20);
% 
% Fz_vec =[];
% Fx_vec =[];
% Fy_vec =[];
% 
% for i = 1:1:length(arrays)
%     [z, Fz, Fx, Fy ,lammda] = nonlinVehicleModel_val(x(i,:),s(i,:)',u(i,:),ax(i),ay(i),mu);
%     Fz_vec = [Fz_vec; Fz];
%     Fx_vec = [Fx_vec; Fx];
%     Fy_vec = [Fy_vec; Fy];
% end
% 
% % figure(); hold on; grid on;
% % plot(Fx_vec(:,1));
% % plot(Fx_vec(:,2));
% % plot(Fx_vec(:,3));
% % plot(Fx_vec(:,4));
% 
% K_fl = 1 - sqrt(Fx_vec(:,1).^2./Fz_vec(:,1).^2/mu^2 + Fy_vec(:,1).^2./Fz_vec(:,1).^2/(mu*0.9)^2);
% K_fr = 1 - sqrt(Fx_vec(:,2).^2./Fz_vec(:,2).^2/mu^2 + Fy_vec(:,2).^2./Fz_vec(:,2).^2/(mu*0.9)^2);
% K_rl = 1 - sqrt(Fx_vec(:,3).^2./Fz_vec(:,3).^2/mu^2 + Fy_vec(:,3).^2./Fz_vec(:,3).^2/(mu*0.9)^2);
% K_rr = 1 - sqrt(Fx_vec(:,4).^2./Fz_vec(:,4).^2/mu^2 + Fy_vec(:,4).^2./Fz_vec(:,4).^2/(mu*0.9)^2);
% K_body = 1- sqrt((ax/(mu*g)).^2 + (ay/(mu*g*0.9)).^2);
% index = zeros(n,1);
% for i = 1:1:n
%     [min_val, index(i)] = min([K_fl(i) K_fr(i) K_rl(i) K_rr(i) K_body(i)]);
% end
% mu = mu;
% K_fl = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2  + Fy_vec(:,1).^2./Fz_vec(:,1).^2/(mu*0.9)^2);
% K_fr = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,2).^2./Fz_vec(:,2).^2/(mu*0.9)^2);
% K_rl = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,3).^2./Fz_vec(:,3).^2/(mu*0.9)^2);
% K_rr = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,4).^2./Fz_vec(:,4).^2/(mu*0.9)^2);
% K_body = 1 - sqrt(((u(:,2)-0.0225*m*g*reff)/(mu*(m+4*Iw/reff^2)*g*reff)).^2 + (0.85*(x(:,6).*x(:,2))./(mu*g*0.9)).^2);
% K_rms = ((K_fl+K_fr+K_rl+K_rr+K_body)/5);
% t = (2:0.001:7);
% plot(t,K_rms(int16(t/0.001)))
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data = load('1207_curve_STenv+inde.mat');
% arrays = data.data.Data;
% Ts = 0.001;
% t_final = length(arrays)*0.001;
% t = (0:Ts:t_final-Ts); n = length(t);
% x = [arrays(:,1)*pi/180 arrays(:,2)*pi/180  arrays(:,3)*pi/180 arrays(:,4) arrays(:,5) arrays(:,6)/3.6];
% ax = arrays(:,7)*9.81;
% ay = arrays(:,8)*9.81;
% s = arrays(:,10:13);
% mu = arrays(1,14);
% u = arrays(:,19:20);
% 
% Fz_vec =[];
% Fx_vec =[];
% Fy_vec =[];
% 
% for i = 1:1:length(arrays)
%     [z, Fz, Fx, Fy ,lammda] = nonlinVehicleModel_val(x(i,:),s(i,:)',u(i,:),ax(i),ay(i),mu);
%     Fz_vec = [Fz_vec; Fz];
%     Fx_vec = [Fx_vec; Fx];
%     Fy_vec = [Fy_vec; Fy];
% end
% 
% % figure(); hold on; grid on;
% % plot(Fx_vec(:,1));
% % plot(Fx_vec(:,2));
% % plot(Fx_vec(:,3));
% % plot(Fx_vec(:,4));
% 
% K_fl = 1 - sqrt(Fx_vec(:,1).^2./Fz_vec(:,1).^2/mu^2 + Fy_vec(:,1).^2./Fz_vec(:,1).^2/(mu*0.9)^2);
% K_fr = 1 - sqrt(Fx_vec(:,2).^2./Fz_vec(:,2).^2/mu^2 + Fy_vec(:,2).^2./Fz_vec(:,2).^2/(mu*0.9)^2);
% K_rl = 1 - sqrt(Fx_vec(:,3).^2./Fz_vec(:,3).^2/mu^2 + Fy_vec(:,3).^2./Fz_vec(:,3).^2/(mu*0.9)^2);
% K_rr = 1 - sqrt(Fx_vec(:,4).^2./Fz_vec(:,4).^2/mu^2 + Fy_vec(:,4).^2./Fz_vec(:,4).^2/(mu*0.9)^2);
% K_body = 1- sqrt((ax/(mu*g)).^2 + (ay/(mu*g*0.9)).^2);
% index = zeros(n,1);
% for i = 1:1:n
%     [min_val, index(i)] = min([K_fl(i) K_fr(i) K_rl(i) K_rr(i) K_body(i)]);
% end
% mu = mu;
% K_fl = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2  + Fy_vec(:,1).^2./Fz_vec(:,1).^2/(mu*0.9)^2);
% K_fr = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,2).^2./Fz_vec(:,2).^2/(mu*0.9)^2);
% K_rl = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,3).^2./Fz_vec(:,3).^2/(mu*0.9)^2);
% K_rr = 1 - sqrt((u(:,2)/(mu*m*g*reff)).^2 + Fy_vec(:,4).^2./Fz_vec(:,4).^2/(mu*0.9)^2);
% K_body = 1 - sqrt(((u(:,2)-0.0225*m*g*reff)/(mu*(m+4*Iw/reff^2)*g*reff)).^2 + (0.85*(x(:,6).*x(:,2))./(mu*g*0.9)).^2);
% K_rms = ((K_fl+K_fr+K_rl+K_rr+K_body)/5);
% t = (8:0.001:11);
% plot(t,K_rms(int16(t/0.001)))

% 
% legend('Proposed','Env','Inde+Env','Bound')
% xlabel('Time [sec]'); ylabel('R_{inst}');
% axis([2 7 -0.6 1])

%% Reference plot
clc; close all;
v_ref = load('v_profile_cur50_0.3_v-s.mat');
v_ref = v_ref.v;
y_ref = load('curve_left_R50_y-s.mat');
x_ref = load('curve_left_R50_x-s.mat');
y_ref = y_ref.y;
x_ref = x_ref.x;

figure();
subplot(1,2,1);
plot(x_ref, y_ref,'-k');  axis([0 110 -10 100]);
xlabel('x_{glo} [m]'); ylabel('y_{glo} [m]');
title('Route reference');
subplot(1,2,2);
plot((1:1:length(v_ref))*0.01, v_ref,'-k')
xlabel('Distance along the path [m]'); ylabel('Velocity [m/s]');
title('Velocity reference');
xlim([0 170]);

%% Animation Moving obstacle
close all; clear; clc;

load('80kph_09_mv50_inputs.mat');
load('80kph_09_mv50_states.mat');
u = inputs.Data;
x = states.Data;

load('80kph_09_mv50_woOW_inputs.mat');
load('80kph_09_mv50_woOW_states.mat');
u_wo = inputs.Data;
x_wo = states.Data;


load('60kph_straight_pu_IRA.mat')
% x = data.Data(:,1:6);

Ts = 0.001;
t_final = 20;
t = (0:Ts:t_final); n = length(t);

load('60kph_straight_pu_off.mat')
x_off = data.Data(:,1:6);
load('60kph_straight_pu_inde.mat')
x_inde = data.Data(:,1:6);

% Vehicle demension
a = 2;
b = 0.9;
r = sqrt(a^2+b^2);
theta = atan2(b,a);

t = int16((0.001:0.001:10)/0.001);

h1 = figure('Color', 'White'); hold on;
p1 = plot(x(:,4),x(:,5),'Color','b','LineWidth',2);
p2 = plot(x(:,4),-2*ones(length(x(:,4)),1),'--r','LineWidth',2);
% 
% p4 = patch([209 209+4.5 209+4.5 209], [-3 -3 -1 -1],'m') ;

x_final =  320;
axis([25 x_final -4 4])
p5 = plot(linspace(0,x_final,200),0*linspace(0,x_final,200),'--','Color','k','LineWidth',3);
p6 = plot(linspace(0,x_final,200),0*linspace(0,x_final,200)-4,'-','Color','k','LineWidth',3);
p7 = plot(linspace(0,x_final,200),0*linspace(0,x_final,200)+4,'-','Color','k','LineWidth',3);
p8 = plot(x_wo(:,4),x_wo(:,5),'r','LineWidth',2);
% p9 = plot(x_inde(:,4),x_inde(:,5),'Color','r','LineWidth',1);
h_obs1 = patch([70+42 70+2*a+42 70+2*a+42 70+42], [-2-b -2-b -2+b -2+b],'m'); % obstacle
% h_obs2 = patch([80 80+2*a 80+2*a 80], [2-b 2-b 2+b 2+b],'m');% obstacle

% legend('Vehicle Path','Reference','Moving Obstacle')
xlabel('x_g_l_o [m]'); ylabel('y_g_l_o [m]');

% patch([99 101 101 99], [50 50 54 54],'m') % obstacle
% for t = [1 ]
for t = 0.001:0.1:t_final
    x_glo = x(int16(t/Ts),4);
    y_glo = x(int16(t/Ts),5);
    yaw = x(int16(t/Ts),3)*pi/180;
    x_ver = [x_glo-r*cos(yaw+theta) x_glo+r*cos(yaw-theta) x_glo+r*cos(yaw+theta) x_glo-r*cos(yaw-theta)];
    y_ver = [y_glo-r*sin(yaw+theta) y_glo+r*sin(yaw-theta) y_glo+r*sin(yaw+theta) y_glo-r*sin(yaw-theta)];
    
    view(2);
%     axis([0 110 -10 100])
    xlabel('x_{glo} [m]'); ylabel('y_{glo} [m]');
    if t== 0.001
        pause(10)
    end
    if t > 0.001
        delete(h_patch1)
%         delete(h_patch2)
    end
    h_patch1 = patch(x_ver, y_ver, 'c');
    x_obs = 68 + t*70/3.6;
   
    legend([p1 p8 p2 h_obs1],{'w/ Priority Factor','w/o Priority Factor','Reference','Obstacle'},'FontSize',12)
    frame1 = getframe(h1);

end    