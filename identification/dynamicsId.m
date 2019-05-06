function dnudt = dynamicsId(nu, U, m, Iz, Xu, Yv, Nr, K)
% DYNAMICSID calculates the function f(X,U,theta) of the dynamic system
% dX/dt = f(t,X,theta) for the parameters theta
%
% Inputs:
%   X1 = x, X2 = y, X3 = psi, X4 = u, X5 = v, X6 = r
%
% Output:
%   dnudt : Derivative of state (velocity) vector


% Ignore all this commented stuff
%eta = X(1:3);   % Position and orientation in ground frame
%nu = X(4:6);    % Linear and angular velocity in body frame

% Parameter setup
% This part can be modularized later -> make setup function

% m = 0.59;       % Mass of hovercraft
% Iz = 0.106;     % Moment of inertia around z axis

% Xu = 0.3;       % Surge damping
% Yv = 0.35;      % Sway damping
% Nr = 0.5E-2;    % Yaw damping
% K = 2;

l = 0.0325;      % Lateral offset of thruster from center line  

% Assumed origin of body frame is at center of gravity

% % Mass matrix
% Minv = [1/m, 0, 0;
%      0, 1/m, 0;
%      0, 0, 1/Iz];
% % Coriolis matrix
% C = [0, 0, - m * nu(2);
%      0, 0, m * nu(1);
%      m * nu(2), - m * nu(1), 0];
% 
% % Damping matrix
% D = diag([Xu, Yv, Nr]);
% 
% % Input matrix
% B = [1, 1;
%      0, 0;
%      l, -l];

dnudt = zeros(3, 1);

% Temporal derivative of x, y, psi
%dnudt(1:3) = R_z_psi * nu;
% Temporal derivative of u, v, r
%dnudt = Minv*(B * U - C * nu - D * nu);

% Decrease computational time by writing all matrix multiplication explicitly
dnudt(1,1) = 1/m*(K*U(1) + K*U(2) + m*nu(2)*nu(3) - Xu*nu(1));
dnudt(2,1) = 1/m*( - m*nu(1)*nu(3) - Yv*nu(2));
dnudt(3,1) = 1/Iz*( K*l*U(1) - K*l*U(2) - m*nu(2)*nu(1) + m*nu(1)*nu(2) - Nr*nu(3) );

end