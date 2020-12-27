function [A,B,C,D,U,Y,X,DX] = vehicleModelDT(x,u,Ts)

% Continuous-Time model
[Ac,Bc] = linearize_model(x,u); % Linearization at the operating point
% Cc = [1 0 0 0 0;
%       0 1 0 0 0;
%       0 0 0 1 0;
%       0 0 0 0 1];
% Dc = [0 0; 0 0; 0 0; 0 0];
Cc = eye(5);
Dc = zeros(5,2);

% Discretize using zero-order hold
nx = size(Ac,1);
nu = size(Bc,2);
M = expm([[Ac Bc]*Ts; zeros(nu,nx+nu)]);
A = M(1:nx, 1:nx);
B = M(1:nx, nx+1:nx+nu);
C = Cc;
D = Dc;

% Nominal conditions of DT plant
X = x;
U = u;
Y = C*x + D*u;
DX = A*x + B*u - x; % x(k+1) - x(k)