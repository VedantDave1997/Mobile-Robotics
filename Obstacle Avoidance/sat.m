function [omega] = sat(komega,steeringDirection,omegamin, omegamax)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

omega = komega*steeringDirection;
if (omega > omegamax)
    omega = omegamax;
elseif(omega < omegamin)
    omega = omegamin;
end
    
end

