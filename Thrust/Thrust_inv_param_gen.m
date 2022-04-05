syms Omega [2 1] real
syms v_in
syms wb [3 1] real
syms Fp [4 1] real
syms rp rho Ith l
%function F = Thrust_inv_param(Omega, v_in, wb, Fp, rp, rho, Ith, l)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
%Omega(1) = Omega_Left, Omega(2) = Omega_Right

%Omega, v_in, wb, Fp, rp, rho, Ith, l
%Omega1 = Omega(1);
%Omega2 = Omega(2);
%wb2 = wb(2);
%wb3 = wb(3);
%Fp1 = Fp(1);
%Fp2 = Fp(2);
%Fp3 = Fp(3);
%Fp4 = Fp(4);

Omega_L = Omega(1);
Omega_R = Omega(2);
J_L = adv_ratio(v_in, Omega_L,rp);
J_R = adv_ratio(v_in, Omega_R, rp);
F = sym(zeros(4,1));
F(1) = expand(combine(collect(Thrust_T(rho, Omega_L, rp, CT_J(J_L)) + Thrust_T(rho, Omega_R, rp, CT_J(J_R)) - Fp(1))));
F(2) = expand(combine(collect(Torque_Q(rho, Omega_R, rp, CP_J(J_R)) - Torque_Q(rho, Omega_L, rp, CP_J(J_L)) - Fp(2))));
F(3) = Ith*(Omega_L - Omega_R)*(-wb(3)) - Fp(3);
F(4) = expand(combine(collect(l * (Thrust_T(rho, Omega_L, rp, CT_J(J_L)) - Thrust_T(rho, Omega_R, rp, CT_J(J_R))) + Ith*(Omega_L - Omega_R)*wb(2) - Fp(4))));

matlabFunction(F, 'File', 'Thrust_inv_param', 'Outputs', {'Thrust_inv_param'});


