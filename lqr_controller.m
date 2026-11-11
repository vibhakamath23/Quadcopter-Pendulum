%% LQR Controller Design
clear; clc; close all
load('linear_system.mat')
A = sys.A;  B = sys.B;  C = sys.C;  D = sys.D;
n = size(A,1);  m = size(B,2);

Q = eye(n);
Q(1:3,1:3) = 50 * Q(1:3,1:3); % position scaling
Q(4:6,4:6) = 25 * Q(4:6,4:6); % velocity scaling
Q(7:9,7:9) = 2.5 * Q(7:9,7:9); % attitude scaling
Q(10:12,10:12) = 0.75 * Q(10:12,10:12); % attitude rate scaling
Q(13,13) = 2 * Q(13,13); % pendulum position scaling
Q(14,14) = 0.5 * Q(14,14); % pendulum velocity scaling

R = eye(m);
R(1,1) = 10 * R(1,1); % total thrust scaling
R(2:4,2:4) = 0.25 * R(2:4,2:4); % rotational velocity scaling
[K, P, E] = lqr(A, B, Q, R); % lqr gain matrix

%% Closed-Loop System
A_cl = A - B*K;
B_cl = B;
C_cl = C;
D_cl = D;

sys_cl = ss(A_cl, B_cl, C_cl, D_cl);

%% Simulate Response
disp('Closed-Loop Poles:');
disp(E);

% all states start at 1
x0 = ones(n, 1); 
t = 0:0.01:10;
[y, t, x] = initial(sys_cl, x0, t);

%% Plot Results
figure;
subplot(2,1,1);
plot(t, x, 'LineWidth', 1.5);
grid on;
title('State Trajectories (Closed-Loop)');
xlabel('Time (s)');
ylabel('States (x)');

subplot(2,1,2);
% control effort: u(t) = -K * x(t)
u = - (K * x')'; 
plot(t, u, 'r', 'LineWidth', 1.5);
grid on;
title('Control Input Effort (u)');
xlabel('Time (s)');
ylabel('Control Action (u)');

save('lqr_controller.mat', 'K')