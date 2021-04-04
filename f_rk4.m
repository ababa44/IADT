function fd=f_rk4(fc , h)
%F_RK4 Runge-Kutta method (RK4)
% 0  |0
% 1/2|1/2
% 1/2|0   1/2
% 1  |0   0   1
% -------------------
%    |1/6 1/3 1/3 1/6
    function x_new=fproto(x,u)
        k1=fc(x,u);
        k2=fc(x+h/2.*k1,u);
        k3=fc(x+h/2.*k2,u);
        k4=fc(x+h.*k3,u);
        x_new=x+h/6*(k1+2*k2+2*k3+k4);
    end
fd=@fproto;
end