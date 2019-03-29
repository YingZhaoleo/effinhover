%% Simulation of hovercraft 
%  Based on surface ship model by Thor I. Fossen, Guidance and Control of Ocean Vehicles

clear 
%close all

% Simulation settings
h = 0.005;            % Sample time
tmax = 30;          % Simulation time   
t = 0:h:tmax;       % Sample times

% Initial state vector
X0 = [0, 0, 0, 0, 0, 0]';

% Input vector
U = 2*max(idinput([length(t), 2], 'prbs')', 0) + [0.2; 0]*ones(1, length(t));
n = round(length(t)/2);
U = U + 10*[0.2 * ones(1, n-1), zeros(1, n); zeros(1, n-1), 0.2 * ones(1, n)];

% Simulate trajectory using ode45 (does not work with the current
% implementation of the dynamic function)
%[T1, X1] = ode45(@(t, y) dynamics(t, y), tspan, X0);
%[T2, X2] = ode23(@(t, y) dynamics(t, y), tspan, X0);

% Simulate using RK4
rk4.name = 'RK4';
% RK4 discrete system = RK4 function with function handle (@dynamics) of dynamics function
rk4.f_discrete = @(X,U) RK4(X, U, h, @dynamics);
rk4.X = X0;
tic
for k = 1:length(t) - 1
  rk4.X(:,k+1) = rk4.f_discrete(rk4.X(:,k), U(:,k));
end
toc

% Simulate using Euler -> write Euler(X, U, h, f) function similar to RK4
euler.name = 'RK4';
euler.f_discrete = @(X,U) EM(X, U, h, @dynamics);
euler.X = X0;
tic
for k = 1:length(t) - 1
  euler.X(:,k+1) = euler.f_discrete(euler.X(:,k), U(:,k));
end
toc


%visualize(t, rk4.X, 20)
%visualize(t, euler.X, 100)

dataId.name = 'Identification data';
dataId.t = t(1:5:end);
dataId.nu = rk4.X(4:6, 1:5:end);
dataId.U = U(:,1:5:end);
dataId.h = h;
dataId.tmax = tmax;
dataId.nu0 = X0(4:6);
save('../identification/dataID.mat', 'dataId')

%[Ures, Ty] = resample(dataId.U', dataId.t, 1/h, 'nearest');
Uq = interp1(dataId.t, dataId.U', t, 'previous');

figure
stairs(t, U(1,:), '-.k')
hold on
stairs(dataId.t, dataId.U(1,:), 'v-b', 'LineWidt', 1)
stairs(t, Uq(:,1)', '*--r', 'LineWidt', 1)


%figure
% hold on
% plot(X1(:,1), X1(:,2), 'b')
% plot(X2(:,1), X2(:,2), 'k--')
% plot(X(1,:), X(2,:), 'r--')
% legend('ode45','ode23','rk4')

