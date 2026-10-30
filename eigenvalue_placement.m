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
desiredPoles = [-2+1i, -2-1i, -2.5, -3, -3.5, -4, -4.5, -5, -5.5, -6, -6.5, -7, -7.5, -8];
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
groups = {1:3,  'Position (m)';
          4:6,  'Velocity (m/s)';
          7:9,  'Attitude (rad)';
          10:12,'Attitude Rate (rad/s)';
          13:14,'Pendulum'};
figure('Name','State Response')
yl = {'m','m/s','rad','rad/s','rad, rad/s'};
for g = 1:size(groups,1)
    subplot(3,2,g)
    plot(t, X(:,groups{g,1}), 'LineWidth', 1.2)
    legend(stateLabels(groups{g,1}), 'Interpreter','latex','Location','best')
    title(groups{g,2}); xlabel('Time (s)'); ylabel(yl{g}); grid on
end

subplot(3,2,6)
c = lines(4);
yyaxis left
h1 = plot(t, U(:,1), 'Color', c(1,:), 'LineWidth', 1.5);
ylabel('u_T', 'Interpreter','latex')
yyaxis right
h2 = plot(t, U(:,2:4), 'LineWidth', 1.2);
set(h2, {'Color'}, num2cell(c(2:4,:), 2));
ylabel('$u_\phi$ - $u_\psi$', 'Interpreter','latex')
xlabel('Time (s)')
grid on
legend([h1; h2], {'$u_T$', '$u_\phi$', '$u_\theta$', '$u_\psi$'}, 'Interpreter','latex', 'Location','best')
title('Control effort u = -Kx');

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
%disp('State-feedback gain matrix:'); disp(K)

%% 7. Parameter Sweep
function [T, peakU, maxTilt] = evalPoles(poles, A, B, x0, xref, tvec, tol)
    n = size(A,1);
    try
        K = place(A, B, poles);
    catch
        T = inf; peakU = inf(1,size(B,2)); maxTilt = inf; return
    end
    Acl = A - B*K;
    e0  = x0 - xref;                                   % valid because A*xref = 0
    E   = initial(ss(Acl, zeros(n,1), eye(n), zeros(n,1)), e0, tvec);
    X   = E + xref.';
    U   = -(K*E.').';
    
    posErr = vecnorm(E(:,1:3), 2, 2);
    idx = find(posErr > tol, 1, 'last');
    if isempty(idx)         
        T = 0;
    elseif idx == numel(tvec)
        T = inf;              % never settled
    else                
        T = tvec(idx+1);
    end
    peakU   = max(abs(U), [], 1);
    maxTilt = max(max(abs(X(:,7:8))));                 % phi, theta
end

tvec = 0:0.005:10;   
tol = 0.05;
Umax = [200 5 5 5];   
tiltMax = deg2rad(90);

scales = linspace(1.5, 8, 30);
T = nan(size(scales));  PU = nan(numel(scales), m);  TL = nan(size(scales));
for i = 1:numel(scales)
    p = -linspace(scales(i), 2.5*scales(i), n);
    [T(i), PU(i,:), TL(i)] = evalPoles(p, A, B, x0, xref, tvec, tol);
end
ok = all(PU <= Umax, 2) & TL.' <= tiltMax;

figure
subplot(1,2,1)
plot(scales, T, 'o-'); 
hold on; 
plot(scales(ok), T(ok), 'go', 'MarkerFaceColor','g')
xlabel('Pole scale s'); 
ylabel('Time to reach target (s)'); 
grid on
legend('all','within limits')
subplot(1,2,2)
plot(scales, PU(:,1), 'o-'); 
yline(Umax(1),'r--');
grid on
xlabel('Pole scale s'); 
ylabel('Peak u_1')