%% Simulation of hovercraft 
%  Based on surface ship model by Thor I. Fossen, Guidance and Control of Ocean Vehicles

clear 
%close all

% Simulation settings
h = 0.005;            % Sample time
tmax = 40;          % Simulation time   
t = 0 :h:tmax;       % Sample times

% Initial state vector
X0 = [0, 0, 0, 0, 0, deg2rad(20)]';

% Input vector
U = [0.2 * ones(1,length(t)); zeros(1, length(t))];
n = round(length(t)/2);
U = [0.2 * ones(1, n), zeros(1, n); zeros(1, n), 0.2 * ones(1, n)];

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

% Simulate using Euler -> write Euler(X, U, h, f) function similar to RK4
% euler.name = 'RK4';
% euler.f_discrete = @(X,U) Euler(X, U, h, @dynamics);
% euler.X = X0;
% for k = 1:length(t) - 1
%   euler.X(:,k+1) = euler.f_discrete(euler.X(:,k), U(:,k));
% end

visualize(t, rk4.X, 100)

%figure
% hold on
% plot(X1(:,1), X1(:,2), 'b')
% plot(X2(:,1), X2(:,2), 'k--')
% plot(X(1,:), X(2,:), 'r--')
% legend('ode45','ode23','rk4')

