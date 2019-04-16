clear all
close all
clc

% The following code i

tmax = 50;                      % Simulation time
h = 0.05;                       % Sample time
t = 0:h:tmax;                   % Time vector
a = 1;                          % Adjacency matrix element
X0 = [0, 0, 0, 0, 0, 0]';       % Initial conditions
U0 = zeros(2,length(t));        % Initialise control input
gamma = 1;                      % Gamma parameter (check paper)
n = 1;                          % eta parameter (check paper)
K = 0.5*eye(2);                 % Control gain

% Set error tolerances and delta matrix
tol1 = 0.1;
tol2 = 0;
tol = [tol1, tol2]';            % Tolerance vector
delta = [1   tol2;              % Delta Matrix
         0  -tol1];

% Simulate with RK4
rk4.name = 'RK4';
rk4.f_discrete = @(X,U) RK4(X, U, h, @dynamics);
rk4.X = X0;
rk4.U = U0;

for k = 1:length(t) - 1
    
    rk4.X(:,k+1) = rk4.f_discrete(rk4.X(:,k), rk4.U(:,k));
    
    % Extract variables
    x = rk4.X(1,:);
    y = rk4.X(2,:);
    psi = rk4.X(3,:);
    u = rk4.X(4,:);
    v = rk4.X(5,:);
    r = rk4.X(6,:);
    
    xd = t(k);                  % Desired x trajectory
    yd = t(k);                  % Desired y trajectory
    
    p = [x; y];                 % Current position
    pd = [xd; yd];              % Desired x-y position
    omega = [0, -r;                 
             r,  0];
    
    % 2x2 rotation matrix (check paper)
    R = [cos(psi(k)),  -sin(psi(k));
         sin(psi(k)),  cos(psi(k))];
     
    vel = [u(k); 0];            % Forward velocity vector (check paper)
    
    pdot = R*vel;               % Update position
    
    e = R'*(p - pd) - tol;
             
    rk4.U = delta' * (delta * delta')^(-1) * R' * pdot - K * e;
end

figure(2)
visualize(t, rk4.X, 100)
title('EM')
