function Omega = fcn(v_in, wb, Fp, rp, rho, Ith, l, Omega_init)
    fun = @(x)Thrust_inv_param(x, v_in, wb, Fp, rp, rho, Ith, l);
    %options = optimoptions('fsolve','Algorithm','levenberg-marquardt','Display','off');
    %x = fsolve(fun, Omega_init, options);
    lb = [0;0];
    [x,res] = lsqnonlin(@fun,Omega_init,lb);
    Omega = x;
end
