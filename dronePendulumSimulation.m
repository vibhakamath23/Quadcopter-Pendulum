function DronePendulumSimulation(states, time, p)
% DronePendulumSimulation simulates the drone with inverted pendulum.
%
% states: Nx14 or 14xN matrix of state histories
% time  : time vector
% p     : parameter structure

    nargin
    if nargin < 3
        p.L = 0.243;
        p.ell = 0.5;
    end

    v = VideoWriter('drone_pendulum.mp4','MPEG-4');
    open(v);

    %------------------------------------------------------------
    % Validate and reshape input
    %------------------------------------------------------------
    states = squeeze(states);
    time   = squeeze(time);

    if size(states,1) == length(time)
        X = states;
    else
        X = states';
    end

    tf = max(time);

    % Extract quantities
    positions   = X(:,1:3);
    eulerAngles = X(:,7:9);
    alpha       = X(:,13);

    assert(size(positions,2) == 3, 'Positions should be Nx3.');
    assert(size(eulerAngles,2) == 3, 'Euler angles should be Nx3.');

    % Interpolate for smoother animation
    dt = 0.002;
    t = 0:dt:tf;
    positions   = interp1(time, positions, t);
    eulerAngles = interp1(time, eulerAngles, t);
    alpha       = interp1(time, alpha, t);

    % Plot bounds
    m = min(positions, [], 1);
    M = max(positions, [], 1);

    figure;
    axis equal
    grid on
    hold on
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Drone with Inverted Pendulum');

    for i = 1:25:length(t)
        clf
        hold on
        grid on

        % Current state
        x   = positions(i,1);
        y   = positions(i,2);
        z   = positions(i,3);

        phi   = eulerAngles(i,1);
        theta = eulerAngles(i,2);
        psi   = eulerAngles(i,3);

        al = alpha(i);

        % Rotation matrix for quadcopter body
        R = eul2rotm_custom([phi, theta, psi]);

        % Pendulum bob position
        xb = x + p.ell*sin(al);
        yb = y;
        zb = z + p.ell*cos(al);

        % Draw quadcopter
        drawDrone(x, y, z, R, p);

        % Draw pendulum rod
        plot3([x xb], [y yb], [z zb], 'k-', 'LineWidth', 2);

        % Draw pendulum bob
        plot3(xb, yb, zb, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);

        % Draw trajectory
        plot3(positions(1:i,1), positions(1:i,2), positions(1:i,3), ...
              '.', 'Color', [0.7 0.7 0.7]);

        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title(sprintf('Drone Movement Simulation, t = %.2f s', t(i)));
        view(3)
        axis equal
        axis([m(1)-1 M(1)+1 m(2)-1 M(2)+1 m(3)-1 M(3)+1])
        drawnow

        frame = getframe(gcf);
        writeVideo(v,frame);
    end

    close(v)
end

%------------------------------------------------------------
function drawDrone(x, y, z, R, p)
    armLength = p.L;
    rotorLength = 0.05;
    rotorRadius = 0.08;

    % Body hub
    plot3(x, y, z, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);

    % Arm endpoints in body frame
    armPoints = [
        0, 0, 0;
        armLength, 0, 0;
        -armLength, 0, 0;
        0, armLength, 0;
        0, -armLength, 0
    ];

    transformed = (R * armPoints')' + [x y z];

    % Draw arms
    for k = 2:5
        line([transformed(1,1), transformed(k,1)], ...
             [transformed(1,2), transformed(k,2)], ...
             [transformed(1,3), transformed(k,3)], ...
             'Color', 'b', 'LineWidth', 2);
    end

    % Draw rotors
    for k = 2:5
        drawRotor(transformed(k,:), R, rotorLength, rotorRadius);
    end
end

%------------------------------------------------------------
function drawRotor(position, R, length, radius)
    [Xc,Yc,Zc] = cylinder(radius, 20);
    Zc(2,:) = length;

    pts = [Xc(:), Yc(:), Zc(:)];
    pts = (R * pts')';
    pts = pts + position;

    surf(reshape(pts(:,1), size(Xc)), ...
         reshape(pts(:,2), size(Yc)), ...
         reshape(pts(:,3), size(Zc)), ...
         'FaceColor', 'g', 'EdgeColor', 'none', 'FaceAlpha', 0.8);
end

%------------------------------------------------------------
function R = eul2rotm_custom(eulerAngles)
    % eulerAngles = [phi theta psi]
    phi = eulerAngles(1);
    theta = eulerAngles(2);
    psi = eulerAngles(3);

    R_x = [1 0 0;
           0 cos(phi) -sin(phi);
           0 sin(phi)  cos(phi)];

    R_y = [ cos(theta) 0 sin(theta);
            0 1 0;
           -sin(theta) 0 cos(theta)];

    R_z = [cos(psi) -sin(psi) 0;
           sin(psi)  cos(psi) 0;
           0 0 1];

    R = R_z * R_y * R_x;
end