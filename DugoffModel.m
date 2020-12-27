function [Fx, Fy, lammda] = DugoffModel(Fz, alpha, s, mu)

% norminal cornering stiffness [N/rad]
% Cf = (1305.3)*180/pi; % Fzf = 4856*2 N
% Cr = (1122.7)*180/pi; % Fzr = 4140*2 N
Cs = [78500 78500 63400 63400];
Ca = [74788 74788 64326 64326];

lammda = mu*Fz.*(1+s)./(2*((Cs.*s).^2+(Ca.*tan(alpha)).^2).^0.5);
 
f = ones(1,4);
for i = (1:1:4)
    if lammda(i) < 1
        f(i) = (2-lammda(i))*lammda(i);
    end
end

Fx = Cs.*s./(1+s).*f;
Fy = Ca.*tan(alpha)./(1+s).*f;
