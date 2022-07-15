function motor
%% Plant 
A = [-0.9250    1.2605   -0.7572
   -1.3609   -0.5701    0.8975
   -0.0100    0.5199   -1.4933];
B = [-0.5945;0.4528;3.6183];
C = [0.2603    2.4954   -0.6639];
%% Augmented Matrix 
discount_fac = 0.01;
F = -1;
T = [A zeros(size(A,1),1);zeros(1,size(A,1)) F];
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
x_old = [0.1;0.1;0.1];
x_next = [0;0;0];
x_old_nw = [0.1;0.1;0.1];
x_next_nw = [0;0;0];
pole = [-0.2 0.2 0.1];
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