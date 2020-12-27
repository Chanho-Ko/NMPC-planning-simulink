clc; clear; close all;

mu = 0.3;
alpha = 1*pi/180*ones(1,4);
s = 0.05*ones(1,4);
Fz = 4000*ones(1,4);

figure(); hold on; grid on;
for deg = -10:0.01:10
    alpha = deg*pi/180.*ones(1,4);
    [Fx, Fy, lammada] = DugoffModel(Fz, alpha, s, mu);
    plot(alpha,Fy(1),'bo')
end
% 
% alpha = 2*pi/180*ones(1,4);
% for deg = -1:0.01:1
%     s = deg*pi/180.*ones(1,4);
%     [Fx, Fy] = DugoffModel(Fz, alpha, s, mu);
%     plot(Fy(1),Fx(1),'go')w
% end
% 
% 
% alpha = 4*pi/180*ones(1,4);
% for deg = -1:0.01:1
%     s = deg*pi/180.*ones(1,4);
%     [Fx, Fy] = DugoffModel(Fz, alpha, s, mu);
%     plot(Fy(1),Fx(1),'ro')
% end
% 
% 
% alpha = 6*pi/180*ones(1,4);
% for deg = -1:0.01:1
%     s = deg*pi/180.*ones(1,4);
%     [Fx, Fy] = DugoffModel(Fz, alpha, s, mu);
%     plot(Fy(1),Fx(1),'co')
% end


xlabel('F_y [N]'); ylabel('F_x [N]'); title('Tire force');
