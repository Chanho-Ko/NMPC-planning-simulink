%% ---------------------------------------------------------------
%                   Nonlinear MPC is designed here
%%-------------------------------------------------------------------------
%                               < DESCRIPTION >
%
%       [States]
%           beta = x(1); % [rad]
%           yawrate = x(2); % [rad/s]
%           yaw = x(3); % [rad]
%           x_glo = x(4); [m]
%           y_glo = x(5); [m]
%   
%       [Input]
%           delta = u(1); % [rad]
%--------------------------------------------------------------------------
clc; clear;

%% Scenario Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

v = 60/3.6; % [m/s]
mu = 0.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% MPC Design

nx = 5; ny = 5; nu = 1;
Ts = 0.05; Hp = 20; Hc =20;
nlobj = nlmpc(nx, ny, nu);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = Hp;
nlobj.ControlHorizon = Hc;
nlobj.Model.NumberOfParameters = 6;
mdl = 'MPC_planner_com1_201125';
t = 0; ax = 0; ay = 0; s = [0 0 0 0]';
createParameterBus(nlobj,[mdl '/Motion Planner/Nonlinear MPC Controller'],'myBusObject',{t, ax, ay, v, mu, s});
% nlobj.Optimization.SolverOptions.MaxIterations = 50;

% Vehicle Model
nlobj.Model.StateFcn = @nonlinVehicleModel_com1;
% nlobj.Jacobian.StateFcn = @myStateJacobian;
nlobj.Model.OutputFcn = @outputFcn_com1;
nlobj.Jacobian.OutputFcn = @myOutputJacobian_com1;

% Custum Cost function
nlobj.Optimization.CustomCostFcn = @myCostFunction_com1;
nlobj.Optimization.ReplaceStandardCost = false;

% Input Constraints
nlobj.MV(1).Min = -45*pi/180;
nlobj.MV(1).Max = 45*pi/180;
nlobj.MV(1).RateMin = -45*pi/180*Ts; % hard constraint of degrees per second
nlobj.MV(1).RateMax = 45*pi/180*Ts;
% nlobj.MV(2).Min = -2000;
% nlobj.MV(2).RateMin = -5000*Ts;
% nlobj.MV(2).Max = 650;
% nlobj.MV(2).RateMax = 5000*Ts;

% Output Contstraints (soft)
beta_ub = atan(0.02*mu*9.81);
yawrate_ub = 0.85*9.81*mu/v;

nlobj.OutputVariables(1).Min = -beta_ub;
nlobj.OutputVariables(1).Max = beta_ub;
nlobj.OutputVariables(2).Min = -yawrate_ub; % 0.85 * 9.81 * mu / v_x [rad/s]
nlobj.OutputVariables(2).Max = yawrate_ub;

nlobj.OutputVariables(1).MaxECR > 0;
nlobj.OutputVariables(1).MinECR > 0;
nlobj.OutputVariables(2).MaxECR > 0;
nlobj.OutputVariables(2).MinECR > 0;
% 
nlobj.Weights.ECR = 5;

% Weights of Cost Function
yaw_error_max = 1.8*pi/180;
pos_error_max = 0.1; % 0.15
% v_error_max = 0.18;
nlobj.Weights.OutputVariables = 0.05*[0 0 1/yaw_error_max^2 1/pos_error_max^2 1/pos_error_max^2];
nlobj.Weights.ManipulatedVariables = 50*[1];
nlobj.Weights.ManipulatedVariablesRate = 0.05*[300];  % nominal input range = [-5~5 500~700]


% Initial Conditions
x0 = [0; 0; 0; 2; 0];
u0 = [0];
validateFcns(nlobj,x0,u0,[],{0, 0, 0, 1, mu, [0.01 0.01 0.01 0.01]'});

% get_param('Base_Model/MPC/Adaptive MPC Control                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  ler','Userdata')
% unob = length(A)-rank(obsv(A,C)) % unobservable states
% nlobj.Model.Nominal = struct('U',U,'Y',Y,'X',X,'DX',DX);
% 
% nlobj.Optimization.SolverOptions.MaxIterations = 10;
nlobj.Optimization.SolverOptions.ConstraintTolerance = 1e-01;
% nlobj.Optimization.SolverOptions.OptimalityTolerance = 1e-03;
% nlobj.Optimization.SolverOptions.StepTolerance = 1e-02;


%% Path Generation

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
