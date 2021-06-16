function [ v, omega ] = homingControl1( rho, alpha, phi )
    Kp=0.5;
    Ka=0.7;
    Kphi=-0.1;
    omegaMax=2;
    Vmax=2;
    v=Kp*rho;
    if(abs(v)>Vmax)
        v=sign(v)*Vmax;
    end
   
    if(abs(alpha)>pi/2) %in I2
  
        v=-v;
        if(sign(alpha)<0)
            alpha=alpha+pi;
        else
            alpha=alpha-pi;
        end
    end

    omega=Ka*alpha+Kphi*phi;
    
    if(abs(omega)>omegaMax)
        omega=sign(omega)*omegaMax;
    end   
%     if(rho<0.01)
%         v=0;
%     end
%     if(alpha<0.03)
%         omega=0;
%     end
%     if(phi<0.03)
%         omega=0;
%     end
end