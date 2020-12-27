function C = myOutputJacobian(x,u,t, ax, ay,index, mu, s)
C = [0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];