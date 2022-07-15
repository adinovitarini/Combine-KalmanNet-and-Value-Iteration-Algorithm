function [P,K,G] = value_iteration(Tbar,N,Bonebar,Q,R,discount_fac)
%Value Iteration Algorithm
% K = rand(1,size(T,1));
P = zeros(size(Tbar,1),size(Tbar,1));
K = zeros(1,size(Tbar,1));
[Pdare,Kdare] = idare(Tbar,Bonebar,Q,R);
for j = 1:N
    if j~=1
    P_new = Q + K'*R*K+discount_fac*((Tbar-Bonebar*K)'*P*(Tbar-Bonebar*K));
    K_new = inv(R+Bonebar'*P_new*Bonebar)*Bonebar'*P_new*Tbar;
    P = P_new;
    K = K_new;
    G(j) = norm(K-Kdare);
    end
end
L = eig(Tbar-Bonebar*K);