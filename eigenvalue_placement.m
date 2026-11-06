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

%% 2. Choose poles (Decoupled by Physical Subsystem)
desiredPoles = [-1.5+1i, -1.5-1i, -2.5, -3, -3.5, -4.5, -4.5, -5, -5.5, -6, -6.5, -30, -35, -40];
K = place(A, B, desiredPoles);

% % The 14-state linear model decouples into 4 independent SISO subsystems:
% %   - z (altitude):          states [3, 6],                input 1 (u_T)
% %   - y (lateral):           states [2, 5, 7, 10],         input 2 (u_phi)
% %   - x + pendulum (long.):  states [1, 4, 8, 11, 13, 14], input 3 (u_theta)
% %   - psi (yaw):             states [9, 12],               input 4 (u_psi)
% % Placing poles per subsystem eliminates cross-axis coupling and lowers ||K||.
% 
% idx_z   = [3, 6];
% idx_y   = [2, 5, 7, 10];
% idx_x   = [1, 4, 8, 11, 13, 14];
% idx_psi = [9, 12];
% 
% % --- Preset A: Sub-Second Fast (Active - <1s settling on all axes, ||K|| = 117 < original 163) ---
% poles_x   = [-3.84+5.68i, -3.84-5.68i, -8.28+7.26i, -8.28-7.26i, -3.52, -5.62];
% poles_y   = [-5.80+3.00i, -5.80-3.00i, -13.0, -18.0];
% poles_z   = [-6.50+4.50i, -6.50-4.50i];
% poles_psi = [-8.0, -10.0];
% 
% % --- Preset B: Ultra-Fast Linear (~0.40s - 0.68s settling, higher gain ||K|| = 509) ---
% % poles_x   = [-9.75+7.50i, -9.75-7.50i, -8.16+9.08i, -8.16-9.08i, -24.57, -3.64];
% % poles_y   = [-7.50+4.00i, -7.50-4.00i, -16.0, -22.0];
% % poles_z   = [-8.20+5.70i, -8.20-5.70i];
% % poles_psi = [-10.0, -12.0];
% 
% % --- Preset C: Balanced / Small-Angle Safe (~0.79s - 1.44s settling, |theta| <= 30 deg, ||K|| = 48) ---
% % poles_x   = [-2.30+4.60i, -2.30-4.60i, -2.60+1.45i, -2.60-1.45i, -5.20, -25.0];
% % poles_y   = [-3.75+1.95i, -3.75-1.95i, -8.50, -12.0];
% % poles_z   = [-4.10+2.85i, -4.10-2.85i];
% % poles_psi = [-5.0, -6.0];
% 
% desiredPoles = [poles_x, poles_y, poles_z, poles_psi];
% 
% K = zeros(m, n);
% K(1, idx_z)   = place(A(idx_z, idx_z),     B(idx_z, 1),   poles_z);
% K(2, idx_y)   = place(A(idx_y, idx_y),     B(idx_y, 2),   poles_y);
% K(3, idx_x)   = place(A(idx_x, idx_x),     B(idx_x, 3),   poles_x);
% K(4, idx_psi) = place(A(idx_psi, idx_psi), B(idx_psi, 4), poles_psi);

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
X_err = X - xref.';
activeCols = max(abs(X_err), [], 1) > 1e-9;
settleIdx = find(any(abs(X_err(:, activeCols)) > 0.02*max(abs(X_err(:, activeCols)),[],1), 2), 1, 'last');
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
xref = [1, 0, 0;
    -1, 0, 0;
    0, 1, 0;
    0, -1, 0;
    0, 0, 1;
    0, 0, -1];
settling_bound = 0.02;

for i = 1:6
    % 1. Create the full 14-element target state vector (3 positions + 11 zeros)
    xTarget_full = [xref(i,:)'; zeros(n - 3, 1)];

    % 2. Solve ODE (Parentheses group K*xTarget_full into an m x 1 vector)
    [tStep, xStep] = ode45(@(t,x) Acl*x + B*(K*xTarget_full), tvec, x0);

    % 3. Calculate input profile cleanly without nested transposes
    uStep = (xTarget_full.' - xStep) * K.';

    % 4. Compute 2% Settling Time
    pos_error = max(abs(xStep(:, 1:3) - xref(i,:)), [], 2);
    idx_settle = find(pos_error > settling_bound, 1, 'last') + 1;

    if isempty(idx_settle)
        t_settle = tStep(1); % Already settled at t=0
    elseif idx_settle > length(tStep)
        t_settle = NaN;      % Did not settle within tvec limit
    else
        t_settle = tStep(idx_settle);
    end

    % 5. Print results safely (Fixed the %% and cleaned up the \n alignment)
    fprintf('Target %d: final position = %s, peak input = %s, time to settle (2%%) = %s\n', ...
        i, mat2str(xStep(end,1:3),3), mat2str(max(abs(uStep),[],1),3), num2str(t_settle));

    % Plot
    plot_lin_sys(tStep, xStep,uStep)
end
