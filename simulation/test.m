% Testing the ode45 solver to simulate the dynamic system


% options = odeset('RelTol',1e-5);
h = 0.1; % time step
tmax = 10;
tspan = 0:h:tmax;

X0 = [0, 0, 0, 0.5, 0.5, deg2rad(10)]';
X0 = [0, 0, 0, 0, 0, 0]';

[T, X] = ode45(@(t, y) dynamics(t, y), tspan, X0);

figure
plot(X(:,1), -X(:,2))
axis equal
