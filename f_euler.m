function fd=f_euler(fc , h)
%F_EULER Euler method (RK1)
% 0|0
% ---
%  |1
    function x_new=fproto(x,u)
        x_new=x+h*fc(x,u);
    end
fd=@fproto;
end