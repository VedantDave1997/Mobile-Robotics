function [v,omega] = homingControlPurePursuit( rho, alpha, phi, deltay)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
curvature = (2*deltay)/(rho^2);
v = 0.5;
omega = curvature*v;
end

