function [ v, omega, turnflag]=avoidObstacle(rmin, phimin, vmax, omegaMax, turnflag, prev_omega)
rturn=0.5;
rstop=0.3;
rsafe=2;

if(rmin<rstop)
    v=0;
elseif((rmin>=rstop) & (rmin<=rsafe))
    v=vmax*((rmin-rstop)/(rsafe-rstop));
else
    v=vmax;
end
if(rmin<rturn)
    omega=omegaMax;
elseif(rmin>=rturn & rmin<=rsafe)
    omega=omegaMax*((rsafe-rmin)/(rsafe-rturn));
else
    omega=0;
end



if(rmin >=rturn)
    omega=-(sign(phimin))*abs(omega);
elseif(rmin<rturn)
    omega=(sign(prev_omega))*omegaMax;
end

turnflag=sign(omega);
end
