function C = myOutputJacobian_com1(x,u,t, ax, ay,index, mu, s)
C = [1 0 0 0 0;
    0 1 0 0 0;
    0 0 1 0 0;
    0 0 0 1 0;
    0 0 0 0 1];