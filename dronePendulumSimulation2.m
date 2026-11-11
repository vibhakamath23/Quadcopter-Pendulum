function dronePendulumSimulation2(states, time, p)
% DronePendulumSimulation
% Animation of quadcopter with inverted pendulum and waypoint markers.

    if nargin < 3
        p.L   = 0.243;
        p.ell = 0.5;
    end

    v = VideoWriter('drone_pendulum2.mp4','MPEG-4');
    open(v);

    %------------------------------------------------------------
    % Reshape input
    %------------------------------------------------------------
    states = squeeze(states);
    time   = squeeze(time);

    if size(states,1) ~= length(time)
        states = states';
    end

    tf = max(time);

    % State history
    pos      = states(:,1:3);
    eul      = states(:,7:9);
    alpha    = states(:,13);

    % Interpolate
    dt    = 0.01;
    t     = 0:dt:tf;
    pos   = interp1(time, pos, t);
    eul   = interp1(time, eul, t);
    alpha = interp1(time, alpha, t);

    % Waypoints for display (if available)
    if isfield(p,'waypointx') && isfield(p,'waypointy') && isfield(p,'waypointz')
        wp = [p.waypointx(:), p.waypointy(:), p.waypointz(:)];
    else
        wp = [];
    end

    %------------------------------------------------------------
    % Figure setup
    %------------------------------------------------------------
    fig = figure('Color','w');
    ax = axes(fig);
    hold(ax,'on');
    grid(ax,'on');
    axis(ax,'equal');
    view(ax, 45, 25);
    xlabel(ax,'X (m)');
    ylabel(ax,'Y (m)');
    zlabel(ax,'Z (m)');
    title(ax,'Quadcopter with Inverted Pendulum');

    % Floor grid
    [Xg,Yg] = meshgrid(-10:1:10, -10:1:10);
    Zg = zeros(size(Xg));
    surf(ax, Xg, Yg, Zg, ...
        'FaceAlpha',0.08, 'EdgeColor',[0.8 0.8 0.8], ...
        'FaceColor',[0.7 0.7 0.7]);

    % Plot all waypoints
    if ~isempty(wp)
        plot3(ax, wp(:,1), wp(:,2), wp(:,3), ...
            'ko', 'MarkerFaceColor',[0.4 0.4 0.4], 'MarkerSize',6);
    end

    % Create placeholders for trail and text
    trail = animatedline(ax, 'Color',[0.6 0.6 0.6], 'LineWidth',1.2);

    timeText   = text(ax, -9, 9, 9, '', 'FontSize',12, 'FontWeight','bold');
    statusText = text(ax, -9, 9, 8.3, '', 'FontSize',11, 'FontWeight','bold', ...
        'Color',[0.2 0.2 0.8]);

    % Axis limits
    axis(ax, [-10 10 -10 10 0 12]);

    %------------------------------------------------------------
    % Animation loop
    %------------------------------------------------------------
    nstep = length(t);
    for k = 1:5:nstep
        clf(fig);
        ax = axes(fig);
        hold(ax,'on');
        grid(ax,'on');
        axis(ax,'equal');
        view(ax, 45, 25);
        xlabel(ax,'X (m)');
        ylabel(ax,'Y (m)');
        zlabel(ax,'Z (m)');
        title(ax,'Quadcopter with Inverted Pendulum');

        % Replot floor grid
        surf(ax, Xg, Yg, Zg, ...
            'FaceAlpha',0.08, 'EdgeColor',[0.8 0.8 0.8], ...
            'FaceColor',[0.7 0.7 0.7]);

        % Replot waypoints
        if ~isempty(wp)
            plot3(ax, wp(:,1), wp(:,2), wp(:,3), ...
                'ko', 'MarkerFaceColor',[0.4 0.4 0.4], 'MarkerSize',6);

            % Highlight current waypoint if available
            if isfield(p,'waypointx')
                % Simple estimate of current waypoint index
                idx = min(length(p.waypointx), 1 + floor((k/nstep)*length(p.waypointx)));
                plot3(ax, wp(idx,1), wp(idx,2), wp(idx,3), ...
                    'ro', 'MarkerFaceColor','r', 'MarkerSize',10);
            end
        end

        % Drone state
        x = pos(k,1);
        y = pos(k,2);
        z = pos(k,3);

        phi   = eul(k,1);
        theta = eul(k,2);
        psi   = eul(k,3);
        al    = alpha(k);

        % Rotation matrix
        R = eul2rotm_custom([phi, theta, psi]);

        % Pendulum bob position
        xb = x + p.ell*sin(al);
        yb = y;
        zb = z + p.ell*cos(al);

        % Trajectory trail
        plot3(ax, pos(1:k,1), pos(1:k,2), pos(1:k,3), ...
            '-', 'Color',[0.6 0.6 0.6], 'LineWidth',1.5);

        % Draw drone
        drawDrone(ax, x, y, z, R, p);

        % Draw pendulum rod
        plot3(ax, [x xb], [y yb], [z zb], 'k-', 'LineWidth',2);

        % Draw pendulum bob
        plot3(ax, xb, yb, zb, 'o', ...
            'MarkerSize',10, ...
            'MarkerFaceColor',[0.1 0.1 0.9], ...
            'MarkerEdgeColor','k');

        % Live time / status
        timeText = text(ax, -3, 3, 2, sprintf('t = %.2f s', t(k)), ...
            'FontSize',12, 'FontWeight','bold');
        statusText = text(ax, -3, 3, 2.7, sprintf('Waypoint %d of %d', ...
            min(length(p.waypointx), max(1,1+floor((k/nstep)*length(p.waypointx)))), ...
            length(p.waypointx)), ...
            'FontSize',11, 'FontWeight','bold', 'Color',[0.2 0.2 0.8]);

        % Dynamic camera centering
        xlim(ax, [x-2.5 x+2.5]);
        ylim(ax, [y-2.5 y+2.5]);
        % zlim(ax, [max(0,z-3) z+6]);
        zlim(ax, [0 z+3]);

        drawnow
        frame = getframe(fig);
        writeVideo(v, frame);
    end

    close(v);
end

%------------------------------------------------------------
function drawDrone(ax, x, y, z, R, p)
    armLength = p.L;

    % Arm endpoints in body frame
    armPoints = [ ...
        0, 0, 0;
        armLength, 0, 0;
        -armLength, 0, 0;
        0, armLength, 0;
        0, -armLength, 0];

    transformed = (R * armPoints')' + [x y z];

    % Draw arms
    for k = 2:5
        line(ax, [transformed(1,1), transformed(k,1)], ...
                 [transformed(1,2), transformed(k,2)], ...
                 [transformed(1,3), transformed(k,3)], ...
                 'Color',[0 0.3 0.9], 'LineWidth',2.5);
    end

    % Body center
    plot3(ax, x, y, z, 'ko', 'MarkerFaceColor',[0.1 0.1 0.1], 'MarkerSize',5);

    % Rotors
    for k = 2:5
        plot3(ax, transformed(k,1), transformed(k,2), transformed(k,3), ...
            'o', 'MarkerSize',7, ...
            'MarkerFaceColor',[0.2 0.8 0.2], ...
            'MarkerEdgeColor','k');
    end
end

%------------------------------------------------------------
function R = eul2rotm_custom(eulerAngles)
    phi = eulerAngles(1);
    theta = eulerAngles(2);
    psi = eulerAngles(3);

    R_x = [1 0 0;
           0 cos(phi) -sin(phi);
           0 sin(phi)  cos(phi)];

    R_y = [cos(theta) 0 sin(theta);
           0 1 0;
          -sin(theta) 0 cos(theta)];

    R_z = [cos(psi) -sin(psi) 0;
           sin(psi)  cos(psi) 0;
           0 0 1];

    R = R_z * R_y * R_x;
end