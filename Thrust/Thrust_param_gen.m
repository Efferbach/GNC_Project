syms Omega [2 1] real
syms v_in
syms wb [3 1] real
syms Fp [4 1] real
syms rp rho Ith l
%function F = Thrust_inv_param(Omega, v_in, wb, Fp, rp, rho, Ith, l)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
%Omega(1) = Omega_Left, Omega(2) = Omega_Right

%
%Omega1 = Omega(1);
%Omega2 = Omega(2);
%wb2 = wb(2);
%wb3 = wb(3);
%Omega, wb, v_in, Ith, l, rho, rp

Omega_L = Omega(1);
Omega_R = Omega(2);
J_L = adv_ratio(v_in, Omega_L,rp);
J_R = adv_ratio(v_in, Omega_R, rp);
F = sym(zeros(4,1));
F(1) = expand(combine(collect(Thrust_T(rho, Omega_L, rp, CT_J(J_L)) + Thrust_T(rho, Omega_R, rp, CT_J(J_R)))));
F(2) = expand(combine(collect(Torque_Q(rho, Omega_R, rp, CP_J(J_R)) - Torque_Q(rho, Omega_L, rp, CP_J(J_L)))));
%F(3) = expand(combine(collect(l * (Thrust_T(rho, Omega_L, rp, CT_J(J_L))))));
%F(4) = expand(combine(collect(l * (Thrust_T(rho, Omega_R, rp, CT_J(J_R))))));
F(3) = Ith*(Omega_L - Omega_R)*(-wb(3));
F(4) = expand(combine(collect(l * (Thrust_T(rho, Omega_L, rp, CT_J(J_L)) - Thrust_T(rho, Omega_R, rp, CT_J(J_R))) + Ith*(Omega_L - Omega_R)*wb(2))));

matlabFunction(F, 'File', 'Thrust_param', 'Outputs', {'Thrust_param'});