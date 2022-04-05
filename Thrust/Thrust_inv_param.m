function Thrust_inv_param = Thrust_inv_param(Omega, v_in, wb, Fp, rp, rho, Ith, l)
%Thrust_inv_param
%    Thrust_inv_param = Thrust_inv_param(Fp1,Fp2,Fp3,Fp4,Ith,Omega1,Omega2,L,RHO,RP,V_IN,WB2,WB3)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    28-Mar-2022 15:16:25

%
Omega1 = Omega(1);
Omega2 = Omega(2);
wb2 = wb(2);
wb3 = wb(3);
Fp1 = Fp(1);
Fp2 = Fp(2);
Fp3 = Fp(3);
Fp4 = Fp(4);

t2 = Omega1.^2;
t3 = Omega2.^2;
t4 = rp.^3;
t5 = rp.^4;
t6 = rp.^5;
et1 = -Fp1+rho.*t2.*t5.*5.438921137920692e-2+rho.*t3.*t5.*5.438921137920692e-2-rho.*rp.^2.*v_in.^2.*pi.^2.*1.038339489966678e-1-Omega1.*rho.*t4.*v_in.*pi.*4.847205425449439e-2;
et2 = Omega2.*rho.*t4.*v_in.*pi.*(-4.847205425449439e-2);
mt1 = [et1+et2;-Fp2-rho.*t2.*t6.*6.734120389652055e-3+rho.*t3.*t6.*6.734120389652055e-3+Omega1.*rho.*t5.*v_in.*pi.*1.883489610898851e-3-Omega2.*rho.*t5.*v_in.*pi.*1.883489610898851e-3];
mt2 = [-Fp3-Ith.*wb3.*(Omega1-Omega2)];
mt3 = [-Fp4+Ith.*Omega1.*wb2-Ith.*Omega2.*wb2+l.*rho.*t2.*t5.*5.438921137920692e-2-l.*rho.*t3.*t5.*5.438921137920692e-2-Omega1.*l.*rho.*t4.*v_in.*pi.*4.847205425449439e-2+Omega2.*l.*rho.*t4.*v_in.*pi.*4.847205425449439e-2];
Thrust_inv_param = [mt1;mt2;mt3];
