%% LQR + Full-Order Observer Design (14-State System)
clear; clc; close all

% Load system (n = 14)
load('linear_system.mat') 
A = sys.A; B = sys.B; C = sys.C; D = sys.D;
n = size(A,1); % 14
m = size(B,2); % control inputs
p = size(C,1); % measured outputs

%% Design LQR Controller (same as lqr_controller.m)
Q_lqr = eye(n);
Q_lqr(1:3,1:3) = 50 * Q_lqr(1:3,1:3); % position scaling
Q_lqr(4:6,4:6) = 25 * Q_lqr(4:6,4:6); % velocity scaling
Q_lqr(7:9,7:9) = 2.5 * Q_lqr(7:9,7:9); % attitude scaling
Q_lqr(10:12,10:12) = 0.75 * Q_lqr(10:12,10:12); % attitude rate scaling
Q_lqr(13,13) = 2 * Q_lqr(13,13); % pendulum position scaling
Q_lqr(14,14) = 0.5 * Q_lqr(14,14); % pendulum velocity scaling

R_lqr = eye(m);
R_lqr(1,1) = 10 * R_lqr(1,1); % total thrust scaling
R_lqr(2:4,2:4) = 0.25 * R_lqr(2:4,2:4); % rotational velocity scaling

K = lqr(A, B, Q_lqr, R_lqr);

%% Verify Observability
if rank(obsv(A, C)) < n
    error('The system is not completely observable! Cannot design full-order observer.');
end

%% Design Full-Order Observer Gain (L)

lqr_poles = pole(ss(A-B*K, B, C, D)); 

% Pole Placement 
observer_poles = 5 * real(lqr_poles) + 1i*imag(lqr_poles); 
L = place(A', C', observer_poles)'; 

%% Augmented 28-State Closed-Loop System
% Combined state vector z = [x; x_hat] (Size: 28x1)
% dz/dt = [ A      -B*K ] * [x    ]
%         [ L*C   A-B*K-L*C ]   [x_hat]
A_augmented = [      A,         -B*K; 
    L*C,    A - B*K - L*C ];

B_augmented = [B; B]; 
C_augmented = [C, zeros(p, n);         % Tracks actual outputs y
    zeros(n, n), eye(n)];   % Tracks estimated states x_hat

sys_total = ss(A_augmented, B_augmented, C_augmented, [D; zeros(n, m)]);

%% 5. Simulate Response with State-Estimation Error
x0 = ones(n, 1);       
x_hat0 = zeros(n, 1);  % Observer starts completely blind at 0
z0 = [x0; x_hat0]; 

t = 0:0.005:10; 
[z_response, t] = initial(sys_total, z0, t);

%% data extraction
% Extract the 14 actual states (first 14 columns)
x_actual = z_response(:, 1:14);

% Extract the 14 estimated states (last 14 columns)
x_estimated = z_response(:, 15:28);

% Calculate control effort driven entirely by the observer estimates
u_effort = -(K * x_estimated')'; 

%% Call Modified Plotting Function
plot_lin_sys_observer(t, x_actual, u_effort, x_estimated);

save('lqr_observer.mat', 'K', 'L')