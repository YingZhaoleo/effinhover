function dXdt = dynamics(X, U, t)
% DYNAMICS calculates the function f(X,U,t) of the dynamic system
% dX/dt = f(X,U,t)
%
% Inputs:
%   X : State (column) vector, X1 = x, X2 = y, X3 = psi, X4 = u, X5 = v, X6 = r
%   U : Input (column) vector, U1 = u1 (left thruster), U2 = u2 (right thruster)
%   t : time (optional)
% Output:
%   dXdt : Derivative of state vector

eta = X(1:3);   % Position and orientation in ground frame
nu = X(4:6);    % Linear and angular velocity in body frame

% Parameter setup
% This part can be modularized later -> make setup function

m = 0.59;       % Mass of hovercraft
Iz = 0.106;     % Moment of inertia around z axis

Xu = 0.3;       % Surge damping
Yv = 0.3;      % Sway damping
Nr = 0.5E-2;    % Yaw damping

K = 1;          % Motor signal to thrust conversion coefficient

l = 0.025;      % Lateral offset of thruster from center line

% Time dependent input (used in previous version)
%     if t < 1
%         U = [0,0]';
%     elseif (t > 1) && (t < 15)
%         U = [0.2,0]';
%     elseif (t > 15) && (t < 30)
%         U = [0, 0.2]';
%     else
%         U = [0,0]';
%     end

% Origin of body frame is assumed at center of gravity

% Mass matrix
M = [m, 0, 0;
     0, m, 0;
     0, 0, Iz];
% Coriolis matrix
C = [0, 0, - m * nu(2);
     0, 0, m * nu(1);
     m * nu(2), - m * nu(1), 0];

% Damping matrix
D = diag([Xu, Yv, Nr]);

% Input matrix
B = [1, 1;
     0, 0;
     l, -l];
 
B = [1, 0;
     0, 0;
     0, 1];

% Rotation matrix from body to ground frame
R_z_psi = [cos(eta(3)), -sin(eta(3)), 0;
           sin(eta(3)),  cos(eta(3)), 0;
                0     ,       0     , 1];

dXdt = zeros(6, 1);

% Temporal derivative of x, y, psi
dXdt(1:3) = R_z_psi * nu;
% Temporal derivative of u, v, r
dXdt(4:6) = M\(B * K * U - C * nu - D * nu);

end