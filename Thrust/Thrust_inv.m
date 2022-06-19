function Omega = Thrust_inv(v_in, wb, Fp, rp, rho, Ith, l, Omega_init, lb, ub)
    fun = @(x)norm(Thrust_inv_param(x, v_in, wb, Fp, rp, rho, Ith, l))^2;
    %options = optimoptions('fsolve','Algorithm','levenberg-marquardt','Display','off');
    %x = fsolve(fun, Omega_init, options);
    Omega = fmincon(fun,Omega_init,[],[],[],[], lb, ub);
end

