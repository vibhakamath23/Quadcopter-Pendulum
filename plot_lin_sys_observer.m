function plot_lin_sys_observer(tStep, xStep, uStep, xHatStep)
stateLabels = {'$x$','$y$','$z$', ...
    '$\dot{x}$','$\dot{y}$','$\dot{z}$', ...
    '$\phi$','$\theta$','$\psi$', ...
    '$p$','$q$','$r$', ...
    '$\alpha$','$\dot{\alpha}$'};

groups = {1:3,  'Position (m)';
    4:6,  'Velocity (m/s)';
    7:9,  'Attitude (rad)';
    10:12,'Attitude Rate (rad/s)';
    13:14,'Pendulum'};

figure('Name','State and Estimation Response')
yl = {'m','m/s','rad','rad/s','rad, rad/s'};

for g = 1:size(groups,1)
    subplot(3,2,g)

    % Plot Actual States (Solid lines)
    h_act = plot(tStep, xStep(:,groups{g,1}), 'LineWidth', 1.2); 
    hold on;

    % Plot Estimated States (Dashed lines, matching colors dynamically)
    h_est = plot(tStep, xHatStep(:,groups{g,1}), '--', 'LineWidth', 1.2); 
    for i = 1:length(h_est)
        set(h_est(i), 'Color', get(h_act(i), 'Color')); % Match color to parent state
    end
    hold off;

    % Create semantic legend entries pairing actual and estimated labels
    labels = stateLabels(groups{g,1});
    legLabels = cell(1, 2*length(labels));
    for i = 1:length(labels)
        legLabels{2*i-1} = labels{i};
        legLabels{2*i}   = [labels{i} ' (Est)'];
    end

    legend(legLabels, 'Interpreter','latex','Location','best', 'NumColumns', 2)
    title(groups{g,2}); xlabel('Time (s)'); ylabel(yl{g}); grid on
end

% Control effort remains focused on u = -K * x_hat
subplot(3,2,6)
c = lines(4);
yyaxis left
h1 = plot(tStep, uStep(:,1), 'Color', c(1,:), 'LineWidth', 1.5);
ylabel('u_T', 'Interpreter','latex')
yyaxis right
h2 = plot(tStep, uStep(:,2:4), 'LineWidth', 1.2);
set(h2, {'Color'}, num2cell(c(2:4,:), 2));
ylabel('$u_\phi$ - $u_\psi$', 'Interpreter','latex')
xlabel('Time (s)')
grid on
legend([h1; h2], {'$u_T$', '$u_\phi$', '$u_\theta$', '$u_\psi$'}, 'Interpreter','latex', 'Location','best')
title('Control effort u = -K \cdot \hat{x}');
end
