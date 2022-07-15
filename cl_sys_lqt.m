function [y,X] = cl_sys_lqt(Tbar,Bonebar,Caug,K_dare,x,r,N)
% x = init_x;
% x = x_hat_next
X = [x(:,1);r(1,1)];
for i = 1:N
    if i~=1
        X(:,i) = (Tbar-Bonebar*K_dare)*X(:,i-1);
        X(:,i) = [x(:,i-1);r(i-1,1)];
        y(i) = Caug*X(:,i-1);
        x(:,i) = x(:,i-1);
    end
end
end