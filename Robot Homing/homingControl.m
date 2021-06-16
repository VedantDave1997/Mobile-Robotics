function [ v, omega ] = homingControl( rho, alpha,phi )
krho=0.5;
kalpha=0.7;
kphi= -0.1;
v=krho*rho;
v_max = 2;
omega_max = 2;
%omega=kalpha*alpha;
if(alpha < -pi/2 & alpha >= -pi)
    v=-v;
    alpha = pi + alpha;
elseif(alpha > pi/2 & alpha <= pi)
    v=-v;
    alpha = -pi + alpha;
end

omega=(kalpha*alpha)+(kphi*phi);

if(omega > omega_max)
    omega = omega_max;
elseif(omega < -omega_max)
    omega = -omega_max;
end

if(v>v_max)
    v=v_max;
elseif(v<-v_max)
    v = -v_max;
end


end
