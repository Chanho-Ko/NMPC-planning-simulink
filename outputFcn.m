function y = outputFcn(x,u,t, ax, ay, index, mu, s)

C_temp = eye(6);
C = C_temp(2:6,:);
y = C*x;