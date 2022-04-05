function Q = Torque_Q(rho, omega, rp, CP)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
Q = 4/(pi^3)*rho*omega^2*rp^5*CP;
end