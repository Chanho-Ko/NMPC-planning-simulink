function J = myCostFunction_com1(X,U,e,data,t, ax, ay, V, mu, s)
%--------------------------------------------------------------------------
% Calculating custom cost function for Nonlinear optimization
% Descriptions from 
% < https://kr.mathworks.com/help/mpc/ug/specify-cost-function-for-nonlinear-mpc.html >
%--------------------------------------------------------------------------
% 
% X:    State trajectory from time k to time k+p, specified as a (p+1)-by-Nx array. 
%       The first row of X contains the current state values, which means that
%       the solver does not use the values in X(1,:) as decision variables during optimization.
% 
% U:    Input trajectory from time k to time k+p, specified as a (p+1)-by-Nu array.
%       The final row of U is always a duplicate of the preceding row; that is, U(end,:) = U(end-1,:). 
%       Therefore, the values in the final row of U are not independent decision variables during optimization.
% 
% e:	Slack variable for constraint softening, specified as a nonnegative scalar. 
%       e is zero if there are no soft constraints in your controller.
%       If you have nonlinear soft constraints defined in your inequality constraint function (Model.CustomIneqConFcn),
%       use a positive penalty weight on e and make them part of the cost function.
% 
% data: Additional signals, specified as a structure with the following fields:
% 
%          --------------------------------------------------------------------------------------------------------------
%          Field               Description
%          --------------------------------------------------------------------------------------------------------------
%          Ts                  Prediction model sample time, as defined in the Ts property of the controller
%          CurrentStates       Current prediction model states, as specified in the x input argument of nlmpcmove
%          LastMV              MV moves used in previous control interval, as specified in the lastmv input argument of nlmpcmove
%          References          Reference values for plant outputs, as specified in the ref input argument of nlmpcmove
%          MVTarget            Manipulated variable targets, as specified in the MVTarget property of an nlmpcmoveopt object
%          PredictionHorizon	Prediction horizon, as defined in the PredictionHorizon property of the controller
%          NumOfStates         Number of states, as defined in the Dimensions.NumberOfStates property of the controller
%          NumOfOutputs        Number of outputs, as defined in the Dimensions.NumberOfOutputs property of the controller
%          NumOfInputs         Number of inputs, as defined in the Dimensions.NumberOfInputs property of the controller
%          MVIndex             Manipulated variables indices, as defined in the Dimensions.MVIndex property of the controller
%          MDIndex             Measured disturbance indices, as defined in the Dimensions.MDIndex property of the controller
%          UDIndex             Unmeasured disturbance indices, as defined in the Dimensions.UDIndex property of the controller
%          --------------------------------------------------------------------------------------------------------------
%       
% J:    Computed cost, returned as a scalar

% vehicle parameters
g = 9.81; reff = 0.353; Iw = 0.9; m = 1650 + 180;
Lw = 1.6; % Track width
lf = 1.4;
lr = 1.65;
h = 0.53;

% Variables Calculation
p = 20;
Ts = 0.05;
% for i=1:p+1
%     Y(i,:) = (eye(8)*X(i,:)')';
% end
delta = U(1:p,data.MVIndex(1));
beta = X(2:p+1,1);
r= X(2:p+1,2);
X_glo = X(2:p+1,4);
Y_glo = X(2:p+1,5);

%--------------------------------------------------------------------------
% Cost Function Calculation
% Refer to 'PotentialField_test.m' for the visualization
%--------------------------------------------------------------------------

%%%%%%%%%%%%%%%%%%%% Obstacle Position %%%%%%%%%%%%%%%%%%%%%

p_obs = [100 50];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%% Popup Obstacle %%%%%%%%%%%%%%%%%%%%%%
% if X_glo(1) > 70
%     if X_glo(1) < p_obs(1,1)
%     J_obs1 = sum(sum(A*exp(-((X_glo - p_obs(1,1)).^2/a_r + (Y_glo - p_obs(1,2)).^2/b).^2)));
%     else
%     J_obs1 = sum(sum(A*exp(-((X_glo - p_obs(1,1)).^2/a_f + (Y_glo - p_obs(1,2)).^2/b).^2)));
%     end
% else
%     J_obs1 = 0;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%% Known Obstacle %%%%%%%%%%%%%%%%%%%%%% 
% if X_glo(1) < p_obs(1,1)
%     J_obs1 = sum(sum(A*exp(-((X_glo - p_obs(1,1)).^2/a_r + (Y_glo - p_obs(1,2)).^2/b).^2)));
% else
%     J_obs1 = sum(sum(A*exp(-((X_glo - p_obs(1,1)).^2/a_f + (Y_glo - p_obs(1,2)).^2/b).^2)));
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% if X_glo(1) < p_obs(2,1)
%     J_obs2 = sum(sum(A*exp(-((X_glo - p_obs(2,1)).^2/a_r + (Y_glo - p_obs(2,2)).^2/b).^2)));
% else
%     J_obs2 = sum(sum(A*exp(-((X_glo - p_obs(2,1)).^2/a_f + (Y_glo - p_obs(2,2)).^2/b).^2)));
% end
% J_obs = J_obs1;

af = 6*sqrt(2)/2; b = 3.3*sqrt(2)/2 + 0.3;
%%%%%%%%%%%%%%%%%%%%% Popup Obstacle %%%%%%%%%%%%%%%%%%%%%%
J_pu = 0;
J_pu2 = 0;
if X(1,5) >= 25 %29.17
    if Y_glo(1) < 52
        d = p_obs(2) - Y_glo(1)-2;
        K_obs = ( (X_glo - p_obs(1)).^2./b^2 + (Y_glo-p_obs(2)).^2./d^2 ).^0.5 - 1;
        J_pu = 0.1*sum(sum(exp(-10*K_obs)./max(K_obs,1e-7)));
        
        K_obs = ( (X_glo - 102).^2./b^2 + (Y_glo-p_obs(2)).^2./30^2 ).^0.5 - 1;
        J_pu2 = 0.1*sum(sum(exp(-10*K_obs)./max(K_obs,1e-7)));
        J_pu = J_pu + J_pu2*10;
    else
        K_obs = ( (X_glo - p_obs(1)).^2./b^2 + (Y_glo-p_obs(2)).^2./af^2 ).^0.5 - 1;
        J_pu = 0.1*sum(sum(exp(-10*K_obs)./max(K_obs,1e-7)));
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%% Moving Obstacle %%%%%%%%%%%%%%%%%%%%%%
% d = V.^2/mu/g;
% v_obs = 50/3.6;
% x_ini = 72;
% x_obs_cur = x_ini +t*v_obs;
% x_obs = (x_obs_cur+v_obs*Ts: v_obs*Ts :x_obs_cur + v_obs*Ts*p)';
% J_mv = 0;
% if X_glo(1) > 1
%     if X_glo(1) < x_obs_cur
%         K_mv = ( (X_glo - x_obs).^2./d.^2 + (Y_glo+2).^2./b^2 ).^0.5 - 1;
%         J_mv = 0.1*sum(sum(exp(-10*K_mv)./max(K_mv,1e-7)));
%     else
%         K_mv = ( (X_glo - x_obs).^2./af.^2 + (Y_glo+2).^2./b^2 ).^0.5 - 1;
%         J_mv = 0.1*sum(sum(exp(-10*K_mv)./max(K_mv,1e-6)));
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%% Stability Cost %%%%%%%%%%%%%%%%%%%%%%%
% s = s(index);
% switch index
%     case 1
%         Fz = (g*lr/2-ax*h/2-1*ay*lr*h/Lw+ax*ay*h^2/g/Lw)*m/(lr+lf);
%         alpha = delta - beta - r*lf./V;
%         Cs = 78500; Ca = 74788;
%     case 2
%         Fz = (g*lr/2-ax*h/2+1*ay*lr*h/Lw-ax*ay*h^2/g/Lw)*m/(lr+lf);
%         alpha = delta - beta - r*lf./V;
%         Cs = 78500; Ca = 74788;
%     case 3
%         Fz = (g*lf/2+ax*h/2-1.2*ay*lf*h/Lw-ax*ay*h^2/g/Lw)*m/(lr+lf);
%         alpha = -beta + r*lr./V;
%         Cs = 63400; Ca = 64326;
%     case 4
%         Fz = (g*lf/2+ax*h/2+1.2*ay*lf*h/Lw+ax*ay*h^2/g/Lw)*m/(lr+lf);
%         alpha = -beta + r*lr./V;
%         Cs = 63400; Ca = 64326;
% end
% 
% lammda = mu*Fz.*(1+s)./(2*((Cs.*s).^2+(Ca.*tan(alpha)).^2).^0.5);
% f = ones(p,1);
% for i = (1:1:p)
%     if lammda(i) < 1
%         f(i) = (2-lammda(i))*lammda(i);
%     end
% end
% Fy = Ca.*tan(alpha)./(1+s).*f;
% s_worst = -0.2;
% K_inst = 1 - sqrt((T/(mu*m*g*reff)).^2 + Fy.^2./Fz.^2/(mu*0.9)^2);
% % J_stability = sum(sum( 10*exp(-20*K_inst)./(max(K_inst,1e-2))));
% J_stability = sum(sum( 50*exp(-4*K_inst)./(max(K_inst,1e-5))));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%% Road & Lane Potential Field %%%%%%%%%%%%%%%%
B = 20;
% J_road = sum(sum(1/2*B*((1./(Y_glo-4)).^2 + (1./(Y_glo+4)).^2).^2));
% J_lane = sum(sum(10*exp(-(Y_glo/1.1).^2)));
J_road_left = 0;
J_road = 0;
if X(1,5) >= 24
    J_road = sum(sum(1/2*B*((1./max((-X_glo+101),1e-5)).^2).^2));
%     J_road_left = sum(sum(1/2*B*((1./max((X_glo-94),1e-5)).^2).^2));
end
J_road = J_road;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Total Cost Function %%%%%%%%%%%%%%%%%%%%

% J =  (J_pu + (J_road + J_lane))*0.01;
% J =  (J_pu + (J_road + J_lane))*0.01;
J = (J_road*2 + J_pu)*0.01;
% 0.1*J_mv + 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

