%% Eigenvalue Placement Controller Design
clear; clc; close all
load('linear_system.mat')
A = sys.A;  B = sys.B;  C = sys.C;  D = sys.D;
n = size(A,1);  m = size(B,2);

stateLabels = {'$x$','$y$','$z$', ...
           '$\dot{x}$','$\dot{y}$','$\dot{z}$', ...
           '$\phi$','$\theta$','$\psi$', ...
           '$p$','$q$','$r$', ...
           '$\alpha$','$\dot{\alpha}$'};

%% 1. Sanity checks before placing anything
fprintf('Open-loop eigenvalues:\n'); disp(eig(A))
fprintf('Controllability rank: %d of %d states\n', rank(ctrb(A,B)), n);
fprintf('Number of inputs: %d\n', m);

%% 2. Choose poles
desiredPoles = [-2+1i, -2-1i, -3+1i, -3-1i, -4+1i, -4-1i, -4.5, -5, -5.5, -6, -6.5, -30, -35, -40];
K = place(A, B, desiredPoles);
Acl = A - B*K;
fprintf('Eigenvalues of A_cl: \n')
disp(sort(eig(Acl)))

%% 3. Define desired position and simulate
xref = zeros(n,1);
xref(1) = 1.0;      % desired x (m)
xref(2) = 2.0;      % desired y (m)
xref(3) = 1.5;      % desired z (m)

fprintf('||A*xref|| = %.2e  (must be ~0 for zero steady-state error)\n', norm(A*xref));

% Initial Conditions
x0 = zeros(n,1);
%x0([1 2 3]) = 0.5;              % position offsets
%x0([7 8 9]) = deg2rad(5);       % attitude offsets

fprintf('Initial position error [x y z]: %s m\n', mat2str(x0(1:3).' - xref(1:3).', 3));

tvec = 0:0.01:10;
[t, X] = ode45(@(t,x) Acl*x + B*K*xref, tvec, x0);
U = -(K*(X - xref.').').';

fprintf('Final position error [x y z]: %s m\n', mat2str(X(end,1:3) - xref(1:3).', 3));

%% 4. Plots grouped by physical meaning
plot_lin_sys(t,X,U)

%% 5. Quantitative metrics
settleIdx = find(any(abs(X) > 0.02*max(abs(X),[],1), 2), 1, 'last');
fprintf('\n--- Response metrics ---\n');
fprintf('Approx. settling time (2%%):  %.2f s\n', t(settleIdx));
fprintf('Peak |u| per input:          %s\n', mat2str(max(abs(U),[],1), 3));
fprintf('Gain norm ||K||:             %.2f\n', norm(K));
[V,~] = eig(Acl);
fprintf('Eigenvector cond. number:    %.1e  (large = sensitive to model error)\n', cond(V));
fprintf('Per-state peak magnitude:\n'); 
disp(table(stateLabels', max(abs(X))', 'VariableNames',{'State','Peak'}))

%% 6. Feedback Gain Matrix
disp('State-feedback gain matrix:'); 
disp(K)
save('eigval_controller.mat', 'K')
%% 7. Model Characterization
% xref = [1, 0, 0;
%         -1, 0, 0;
%         0, 1, 0;
%         0, -1, 0;
%         0, 0, 1;
%         0, 0, -1];
% settling_bound = 0.02;
% 
% for i = 1:6
%     % 1. Create the full 14-element target state vector (3 positions + 11 zeros)
%     xTarget_full = [xref(i,:)'; zeros(n - 3, 1)]; 
% 
%     % 2. Solve ODE (Parentheses group K*xTarget_full into an m x 1 vector)
%     [tStep, xStep] = ode45(@(t,x) Acl*x + B*(K*xTarget_full), tvec, x0);
% 
%     % 3. Calculate input profile cleanly without nested transposes
%     uStep = (xTarget_full.' - xStep) * K.'; 
% 
%     % 4. Compute 2% Settling Time 
%     pos_error = max(abs(xStep(:, 1:3) - xref(i,:)), [], 2); 
%     idx_settle = find(pos_error > settling_bound, 1, 'last') + 1;
% 
%     if isempty(idx_settle)
%         t_settle = tStep(1); % Already settled at t=0
%     elseif idx_settle > length(tStep)
%         t_settle = NaN;      % Did not settle within tvec limit
%     else
%         t_settle = tStep(idx_settle);
%     end
% 
%     % 5. Print results safely (Fixed the %% and cleaned up the \n alignment)
%     fprintf('Target %d: final position = %s, peak input = %s, time to settle (2%%) = %s\n', ...
%         i, mat2str(xStep(end,1:3),3), mat2str(max(abs(uStep),[],1),3), num2str(t_settle));
% 
%     % Plot
%     plot_lin_sys(tStep, xStep,uStep)
% end
