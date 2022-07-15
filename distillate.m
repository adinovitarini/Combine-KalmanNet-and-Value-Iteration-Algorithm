function distillate
%% Plant 
A = [1.1367 -0.77978 -0.41183 -0.93463;
    1.0468 1.0221 0.51514 0.55115;
    -0.77322 0.73872 -0.83021 0.0026816;
    1.1816 0.95094 -0.65203 -0.78764];
B = [-1.3696;0.45253;1.0801;-0.37804];
C = [0.72552 -0.78382 0.97289 -0.3413];
%% Augmented Matrix 
discount_fac = 0.01;
F = -1;
T = [A zeros(4,1);zeros(1,4) F];
Tbar = sqrt(discount_fac)*T;
Bone = [B;0];
Bonebar = sqrt(discount_fac)*Bone;
Caug = [C 1];
Q = 10;
R = .01;
N = 100;
%%  Reference signal (sinusoidal)
r=zeros(N,1);
r(1,1) = 1;
x_old = [0.1;0.1;0.1;0.1];
x_next = [0;0;0;0];
x_old_nw = [0.1;0.1;0.1;0.1];
x_next_nw = [0;0;0;0];
pole = [-0.2 0.2 0.1 -0.1];
K_first = acker(A,B,pole);
w = wgn(N,1,1);
v = w;
for i = 1:N
    x_next(:,i) = (A-B*K_first)*x_old(:,i)+w(i);
    x_next_nw(:,i) = (A-B*K_first)*x_old_nw(:,i);
    y(i) = C*x_old_nw(:,i)+v(i);
    x_old(:,i+1) = x_next(:,i);
    x_old_nw(:,i+1) = x_next_nw(:,i);
    r(i+1,1) = -1*r(i,1);
end
x_old = x_old(:,1:N);
x_old_nw = x_old_nw(:,1:N);
%%  KalmanNet 
[x_hat_next,y_hat,KG] = KalmanNet(x_next_nw,y,A,C);
for i=1:size(x_hat_next(:,1))
    x_hat_next(i,:) = normalize(x_hat_next(i,:));
end
%%  Value Iteration 
[P_vi,K_vi,G_vi] = value_iteration(Tbar,N,Bonebar,Q,R,discount_fac);
disp(P_vi)
%%  Close Loop 
[y_vi,~] = cl_sys_lqt(Tbar,Bonebar,Caug,K_vi,x_hat_next,r,N);
%%  Plot 
figure(1);clf
plot(normalize2ref(y_vi,r),'r','LineWidth',2);
hold on
plot(r,'--k')
xlabel('time(sec)');
legend('Output signal','Reference signal')
grid on 
figure(2);clf
plot(G_vi,'k')
end