%% Velocity Profile
close all; 
mu = 0.9; g = 9.81;
R = 50;
ax_des = 3;
vx_lim = (((0.9*mu*g)^2 - ax_des^2)*R^2)^0.25;
vx_lim*3.6
dx = 0.01;
x_final = 300;
xx = (0:dx:x_final);
Ts = 0.05;
ax_des = 3;
v_init = 60/3.6;
v = v_init*ones(length(xx),1);
v(50/dx:150/dx) = vx_lim;
plot(xx,v)

%% Velocity Profile v-s
close all; clc; clear;
ds = 0.01;
s1 = 50;
s2 = sqrt(100^2 + 50^2);
s3 = sqrt(100^2 + 200^2);
v = 60/3.6*ones(int16(s3/ds),1);
mu = 0.3; g = 9.81; Ts = 0.05;
R = 50;
ax_des = 1;
vx_lim = (((0.9*mu*g)^2 - ax_des^2)*R^2)^0.25;
vx_lim*3.6

v(int16((s1:ds:s2)/ds)) = 35/3.6;
ax = 0.5;
i = int16(s1/ds);
while (v(i+1) < 60/3.6)
   v(i-1) = v(i) + ax*Ts/v(i);
   i = i-1;
end
i = int16(s2/ds);
while (v(i-1) < 60/3.6)
   v(i+1) = v(i) + ax*Ts/v(i);
   i = i+1;
end
plot((0:ds:s3),v)
%% Path Generation
close all; clear; clc;

%%% Spline Curve %%%
% way_point = [(0:10:80)' -2*ones(9,1);
%             90 0;
%             100 1;
%             110 1;
%             120 0;
%             130 -1;
%             (140:10:300)' -2*ones(17,1)];  
% x = way_point(:,1);
% y = way_point(:,2);
% 
% xx = (0:0.01:500);
% s = spline(x,y,xx);
% [path_info, gof] = path_spline(x, y);

%%% Straight Lane %%%
s = -2*linspace(1,1,50001);

%% % Right & Left Curves R = 50 %%%
dx = 0.01;
x_final = 300;
xx = (0:dx:x_final);
y = zeros(length(xx),1);
y_cur_right = sqrt(50^2 - (0:0.01:50).^2) - 50;
y_cur_left = -sqrt(50^2 - (-50:0.01:0).^2) - 50;
y(50/dx:100/dx) = y_cur_right;
y(100/dx:150/dx) = y_cur_left;
y(150/dx:end) = -100;
plot(xx,y)
%% % Right & Left Curves R = 25 %%%
clc; close all;
dx = 0.01;
x_final = 300;
xx = (0:dx:x_final);
y = zeros(length(xx),1);
y_cur_right = sqrt(25^2 - (0:0.01:25).^2) - 25;
y_cur_left = -sqrt(25^2 - (-25:0.01:0).^2) - 25;
y(50/dx:75/dx) = y_cur_right;
y(75/dx:100/dx) = y_cur_left;
y(100/dx:end) = -50;

plot(xx,y)
xlim([0 150])

%% Left Curves R = 50 %%%
close all; clc;
dx = 0.01;
x_final = 200;
xx = (0:dx:x_final)';
y = zeros(length(xx),1);
y_cur_left1 = -sqrt(50^2 - (0:0.01:50).^2) + 50;
y_cur_left2 = flip((sqrt(50^2 - (0:0.01:50).^2) + 50));
y(50/dx:100/dx) = y_cur_left1;
y(100/dx+1:150/dx+1) = y_cur_left2;
y(150/dx+1:end) = 100;
y_u = flip(y(100/dx+1:end));
% x = zeros(length(xx),1);
% x_cur_left1 = -sqrt(50^2 - (0:0.01:50).^2) + 50;
% x_cur_left2 = flip(sqrt(50^2 - (0:0.01:50).^2) + 50);
% x(50/dx:100/dx) = x_cur_left1;
% x(100/dx:150/dx) = x_cur_left2;
% x(150/dx:end) = 100;
% 
% s = zeros(length(xx)*2,1);
% s(1:length(xx)) = sqrt(xx.^2 + y1.^2);
% s(length(xx)+1:end) = sqrt(flip(xx).^2 + flip(y2).^2);
% 
% yaw = zeros(length(xx)*2,1);
% yaw(50/dx+1:150/dx)=linspace(0,180*pi/180, 100/dx);
% yaw(150/dx:end) = 180*pi/180;
% plot(xx,y1); hold on;
% plot(xx,y2)
% figure()
% plot(yaw)
% 
% figure();
% plot(s)
% 
% figure();
% plot([s(2:end); s(end)]-s.*sin(yaw))

plot(xx,y); hold on;
plot(xx(1:100/dx+1),y_u)

%% Curve y-s function
close all; clear; clc;
R = 50;
d = -6;

ds = 0.01;
s1 = sqrt(50^2 + d^2);
s2 = sqrt((100+d)^2 + 50^2);
s3 = sqrt((100+d)^2 + 200^2);
y = zeros(int16(s3/ds),1);


% left curve with R = 50
y_cur = -d:0.01:R;
x_cur = 50:0.01:50+R+d;
syms S Y
eqn = (((R+d).^2-(Y-50).^2).^0.5 + 50).^2 +Y.^2 - S.^2 == 0;
sol = solve(eqn,Y);
S = (s1:ds:s2);
Y1 = S.^2/200 - (- S.^4 + 13872*S.^2 -9388096).^(1/2)/200 + 383/25;

y(int16(S/ds)) = Y1;

% Straight to North
S = (s2:ds:s3);
Y = (S.^2 - (100+d)^2).^(1/2);
y(int16(S/ds)) = Y;
y(1:int16(s1/ds))= -d;
y(end) = 200;

figure()
plot(y)

S = (0:ds:s3)';
x = (S.^2 - y.^2).^(1/2);
figure()
plot(x)

figure()
plot(x,y)
axis equal;