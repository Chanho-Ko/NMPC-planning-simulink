function [A,B]=linearize_model(x,u,t,ax,ay,mu)
% ---------------Options--------------------
nu=size(u,1); nx=size(x,1);
delta = 1e-5;
upert = delta+1e-3*delta*abs(u);
xpert = delta+1e-3*delta*abs(x);

% Initialization of nominal outputs and derivatives
oldx=x; oldu=u;
% force all rates in the model to have a hit
dx = nonlinVehicleModel(x,u,t,ax,ay,mu);
olddx=dx;

% Initialize the state terms
A=zeros(nx,nx); B=zeros(nx,nu);

% A matrix
for i=1:nx
    x(i) = x(i)+xpert(i);
    dx = nonlinVehicleModel(x,u,t,ax,ay,mu);
    A(:,i)=(dx-olddx)./xpert(i);
    x=oldx;
end

% B matrix
for i=1:nu
    u(i)=u(i)+upert(i);
    dx = nonlinVehicleModel(x,u,t,ax,ay,mu);
    B(:,i)=(dx-olddx)./upert(i);
    u=oldu;
end