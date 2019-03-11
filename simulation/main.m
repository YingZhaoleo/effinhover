%% Simulation of hovercraft 
%  Based on surface ship model by Thor I. Fossen, Guidance and Control of Ocean Vehicles

% Simulation settings
h = 0.01;           % time step
tmax = 40;          % simulation time   
tspan = 0:h:tmax;

% Initial condition
X0 = [0, 0, 0, 0, 0, 0]';

% Calculate trajectory
[T, X] = ode45(@(t, y) dynamics(t, y), tspan, X0);

visualize(T, X, 10)
