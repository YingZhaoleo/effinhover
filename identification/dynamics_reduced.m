function dnudt = dynamics_reduced(nu, U, param)
% DYNAMICSID calculates the function f(X,U,theta) of the dynamic system
% dX/dt = f(t,X,theta) for the parameters theta
%
% Inputs:
%   X1 = x, X2 = y, X3 = psi, X4 = u, X5 = v, X6 = r
%
% Output:
%   dnudt : Derivative of state (velocity) vector

% parameters of hovercraft
m = param(1);      % mass of hovercraft
Iz = param(2);     % moment of inertia around z axis
Xu = param(3);     % surge damping
Yv = param(4);     % sway damping
Nr = param(5);     % yaw damping
K = param(6);      % motor signal to thrust conversion coefficient
l = param(7);      % lateral offset of thruster from center line

dnudt = zeros(3, 1);

% Decrease computational time by writing all matrix multiplication explicitly
dnudt(1,1) = 1/m*(K*(U(1) + U(2)) + m*nu(2)*nu(3) - Xu*nu(1));
dnudt(2,1) = 1/m*( -m*nu(1)*nu(3) - Yv*nu(2));
dnudt(3,1) = 1/Iz*( K*l*(U(1) - U(2)) - Nr*nu(3) );

end