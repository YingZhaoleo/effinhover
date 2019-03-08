function dXdt = dynamics(t, X)
% DYNAMICS calculates the function f(t,X,u) of the dynamic system
% dX/dt = f(t,X,u)
%
% Inputs:
% X1 = x, X2 = y, X3 = psi, X4 = u, X5 = v, X6 = r

    eta = X(1:3);   % Position and orientation in ground frame
    nu = X(4:6);    % Linear and angular velocity in body frame

    % Parameter setup
    % This part can be modularized later -> make setup function

    m = 0.59;       % Mass of hovercraft
    Iz = 0.106;     % Moment of inertia around z axis

    Xu = 0.3;       % Surge damping
    Yv = 0.35;      % Sway damping
    Nr = 0.5E-2;    % Yaw damping

    l = 0.02;      % Lateral offset of thruster from center line

    % Funky input
    input = [max(0,square(t*2, 20)), max(0,square(t*2 + pi, 20))]';

    % Assumed origin of body frame is at center of gravity

    % Mass matrix
    M = [m, 0, 0;
         0, m, 0;
         0, 0, Iz];
    % Coriolis matrix
    C = [0, 0, - m * nu(2);
         0, 0, m * nu(1);
         m * nu(2), - m * nu(1), 0];

    % Damping matrix
    D = - diag([Xu, Yv, Nr]);

    % Input matrix
    B = [1, 1;
         0, 0;
         l, -l];
     
    % Rotation matrix from body to ground frame
    R_z_psi = [cos(eta(3)), -sin(eta(3)), 0;
              sin(eta(3)), cos(eta(3)), 0;
                    0     ,       0    , 1];

    dXdt = zeros(6, 1);

    % Temporal derivative of x, y, psi
    dXdt(1:3) = R_z_psi * nu;
    dXdt(4:6) = - M\(- B * input - C * nu - D * nu);

end