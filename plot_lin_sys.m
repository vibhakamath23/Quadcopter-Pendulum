function plot_lin_sys(tStep,xStep,uStep)
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
    figure('Name','State Response')
    yl = {'m','m/s','rad','rad/s','rad, rad/s'};
    for g = 1:size(groups,1)
        subplot(3,2,g)
        plot(tStep, xStep(:,groups{g,1}), 'LineWidth', 1.2)
        legend(stateLabels(groups{g,1}), 'Interpreter','latex','Location','best')
        title(groups{g,2}); xlabel('Time (s)'); ylabel(yl{g}); grid on
    end

    subplot(3,2,6)
    c = lines(4);
    yyaxis left
    h1 = plot(tStep, uStep(:,1), 'Color', c(1,:), 'LineWidth', 1.5);
    ylabel('$u_T$', 'Interpreter','latex')
    yyaxis right
    h2 = plot(tStep, uStep(:,2:4), 'LineWidth', 1.2);
    set(h2, {'Color'}, num2cell(c(2:4,:), 2));
    ylabel('$u_\phi$ - $u_\psi$', 'Interpreter','latex')
    xlabel('Time (s)')
    grid on
    legend([h1; h2], {'$u_T$', '$u_\phi$', '$u_\theta$', '$u_\psi$'}, 'Interpreter','latex', 'Location','best')
    title('Control effort u = -Kx');
end