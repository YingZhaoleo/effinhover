%% Simulation of hovercraft 
%  Based on surface ship model by Thor I. Fossen, Guidance and Control of Ocean Vehicles

clear 
%close all

% Simulation settings
h = 0.005;           % Sample time
tmax = 25;           % Simulation time   
t = 0 :h:tmax;       % Sample times

% Initial state vector
X0 = [0, 0, 0, 0, 0, 0]';

% Input vector
U = 2*max(idinput([length(t), 2], 'prbs')', 0) + [0.2; 0]*ones(1, length(t));

% Simulate trajectory using ode45 (does not work with the current
% implementation of the dynamic function)
%[T1, X1] = ode45(@(t, y) dynamics(t, y), tspan, X0);
%[T2, X2] = ode23(@(t, y) dynamics(t, y), tspan, X0);

% Simulate using RK4
rk4.name = 'RK4';
% RK4 discrete system = RK4 function with function handle (@dynamics) of dynamics function
rk4.f_discrete = @(X,U) RK4(X, U, h, @dynamics);
rk4.X = X0;
for k = 1:length(t) - 1
  rk4.X(:,k+1) = rk4.f_discrete(rk4.X(:,k), U(:,k));
end

% % Simulate using EM (Euler Method)
% em.name = 'Euler Method';
% em.f_discrete = @(X,U) EM(X, U, h, @dynamics);
% em.X = X0;
% for k = 1:length(t) - 1
%   em.X(:,k+1) = em.f_discrete(em.X(:,k), U(:,k));
% end

visualize(t, rk4.X, 100)

dataId.name = 'Identification data';
dataId.nu = rk4.X(4:6, :);
dataId.U = U;
dataId.h = h;
dataId.tmax = tmax;
dataId.nu0 = X0(4:6);
save('../identification/dataID.mat', 'dataId')

%figure
% hold on
% plot(X1(:,1), X1(:,2), 'b')
% plot(X2(:,1), X2(:,2), 'k--')
% plot(X(1,:), X(2,:), 'r--')
% legend('ode45','ode23','rk4')