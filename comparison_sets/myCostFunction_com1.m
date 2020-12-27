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
p_obs1 = [70 -2.15];
p_obs2 = [80 1.85];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

af = 6*sqrt(2)/2; b = 2.8*sqrt(2)/2;

%%%%%%%%%%%%%%%%%%%%% Popup Obstacle %%%%%%%%%%%%%%%%%%%%%%
J_pu = 0;
% if X_glo(1) >= 70 - 20 - 2 % TTC = 2
%     if X_glo(1) < p_obs1(1)
%         d = p_obs1(1) - X_glo(1)- 2;
% %         d = 1;
%         K_obs = ( (X_glo - p_obs1(1)).^2./d^2 + (Y_glo - p_obs1(2)).^2./b^2 ).^0.5 - 1;
%         J_pu = 0.1*sum(sum(exp(-10*K_obs)./max(K_obs,1e-3)));
%     else
%         K_obs = ( (X_glo - p_obs1(1)).^2./af^2 + (Y_glo - p_obs1(1)).^2./b^2 ).^0.5 - 1;
%         J_pu = 0.1*sum(sum(exp(-10*K_obs)./max(K_obs,1e-3)));
%     end
% end
if X_glo(1) >= 70 - 20 - 2 % TTC = 2
    if X_glo(1) < p_obs1(1)
        d = p_obs1(1) - X_glo(1)- 2;
%         d = 1;
        K_obs = ( (X_glo - p_obs1(1)).^2./d^2 + (Y_glo - p_obs1(2)).^2./b^2 ).^0.5 - 1;
        J_pu = sum(sum(exp(-0.1*K_obs)./max(K_obs,1e-2)));
    else
        K_obs = ( (X_glo - p_obs1(1)).^2./af^2 + (Y_glo - p_obs1(1)).^2./b^2 ).^0.5 - 1;
        J_pu = sum(sum(exp(-0.1*K_obs)./max(K_obs,1e-2)));
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%% Known Obstacle %%%%%%%%%%%%%%%%%%%%%% 
% if X_glo(1) < p_obs2(1)
%     d = 5;
%     K_obs = ( (X_glo - p_obs2(1)).^2./d^2 + (Y_glo - p_obs2(2)).^2./b^2 ).^0.5 - 1;
%     J_kn = 0.1*sum(sum(exp(-10*K_obs)./max(K_obs,1e-3)));
% else
%     K_obs = ( (X_glo - p_obs2(1)).^2./af^2 + (Y_glo - p_obs2(1)).^2./b^2 ).^0.5 - 1;
%     J_kn = 0.1*sum(sum(exp(-10*K_obs)./max(K_obs,1e-3)));
% end
if X_glo(1) < p_obs2(1)
    d = 5;
    K_obs = ( (X_glo - p_obs2(1)).^2./d^2 + (Y_glo - p_obs2(2)).^2./b^2 ).^0.5 - 1;
    J_kn = sum(sum(exp(-0.1*K_obs)./max(K_obs,1e-2)));
else
    K_obs = ( (X_glo - p_obs2(1)).^2./af^2 + (Y_glo - p_obs2(1)).^2./b^2 ).^0.5 - 1;
    J_kn = sum(sum(exp(-0.1*K_obs)./max(K_obs,1e-2)));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% Road & Lane Potential Field %%%%%%%%%%%%%%%%
B = 20;
J_road = sum(sum(1/2*B*((1./(Y_glo-4)).^2 + (1./(Y_glo+4)).^2).^2));
J_lane = sum(sum(10*exp(-(Y_glo/1.1).^2)));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% Total Cost Function %%%%%%%%%%%%%%%%%%%%

J = ((J_pu + J_kn))*10;
%  + J_road + J_lane
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


