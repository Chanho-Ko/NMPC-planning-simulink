function y = outputFcn(x,u,t, ax, ay, index, mu, s)

C_temp = eye(5);
C = C_temp(1:5,:);
y = C*x;