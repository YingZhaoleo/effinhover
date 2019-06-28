%% Simulation of hovercraft 
%  Based on surface ship model by Thor I. Fossen, Guidance and Control of Ocean Vehicles

clear 
close all

%% simulation settings
h = 0.005;          % sample time
tmax = 30;          % simulation time   
t = 0:h:tmax;       % time vector

%% hovercraft parameters

hovercraft = 'MVWT';     % TinyWhoover or MVWT (reference paper)

switch (hovercraft)
    case 'TinyWhoover'
        m = 0.0583;      % mass
        Iz = 91e-6;      % moment of inertia around z axis
        Xu = 31e-3;      % linear damping
        Nr = 26e-6;      % yaw damping
        k_th = 0.06;     % thrust conversion coefficient
        l = 0.0325;      % lateral offset of thruster from center line
    case 'MVWT'
        m = 5.15;        % mass
        Iz = 0.047;      % moment of inertia around z axis
        Xu = 4.5;        % linear damping
        Nr = 0.41;       % yaw damping
        k_th = 1;        % thrust conversion coefficient
        l = 0.123;       % lateral offset of thruster from center line
end

param = [m, Iz, Xu, Xu, Nr, k_th, l];

%% desired trajectory
switch (1)
    case 1
        % circle
        R = 1.5;                  % radius of circle
        w = 2.0 * pi / 10.0;    % angular velocity
        w = 0.6;
        pd = R * [cos(w*t); sin(w*t)];
        pd_d = R*w * [-sin(w*t); cos(w*t)];
        pd_dd = R*(w)^2 * [-cos(w*t); -sin(w*t)];
        pd_ddd = R*(w)^3 * [sin(w*t); -cos(w*t)];
    case 2
        % cosine
        w = 2 * pi / 10;
        w = 0.3;
        pd = [cos(w*t); t/3];
        pd_d = [-w*sin(w*t); ones(size(t))/3];
        pd_dd = [(2*pi*0.05)^2 * cos(w*t); zeros(size(t))];
        pd_ddd = [-(2*pi*0.05)^3 * sin(w*t); zeros(size(t))];    
    case 3
        % parabola
        pd = [t/4; t.*t/20];
        pd_d = [ones(size(t))/4; t/10];
        pd_dd = [zeros(size(t)); ones(size(t))/10];
        pd_ddd = zeros(size(pd));
    case 4
        % rose curve
        k = 3/2;
        pd = [cos(k*t).*cos(t); cos(k*t).*sin(t)];
        pd_d = [-k*sin(k*t).*cos(t) - cos(k*t).*sin(t); ...
                -k*sin(k*t).*sin(t) + cos(k*t).*cos(t)];
        pd_dd = [ 2*k*sin(t).*sin(k*t) - (k^2 + 1)*cos(t).*cos(k*t); ...
                 -(k^2 + 1)*sin(t).*cos(k*t) - 2*k*cos(t).*sin(k*t)];
        pd_ddd = zeros(size(pd));   % too lazy to calculate 3rd derivative

    case 5
        % straight line
        speed = 0.5;        % speed in 1/s
        p0 = [-2.5; 2.5];
        p1 = 2 * [2.5; -2.5];
        pd = p0.*(1 - speed*t) + p1.*speed.*t;
        pd_d = ones(size(pd))*speed.*(p1 - p0);
        pd_dd = zeros(size(pd));
        pd_ddd = zeros(size(pd));
        
        % stay at the endpoint after reaching it
        ind = pd(1,:) > p1(1);
        pd(:, ind) = ones(2, length(ind(ind == 1))).*p1;
        pd_d(:, ind) = 0;
end


%% controller parameters

switch (hovercraft)
    case 'TinyWhoover'
        ke = 0.1;
        kphi = 0.005 * eye(2);
        kz = 0.0008;
        delta = -0.01*[0.1; 0.0]; 
    case 'MVWT'
        ke = 4.0;          
        kphi = 1.5 * eye(2);
        kz = 0.6;
        delta = -0.1*[0.1; 0]; 
end

%% simulate system using RK4

% initial state vector
X0 = zeros(6, 1);
X0(1:2) = [-3, 2]

% initial controller input
U = [0; 0];

rk4.name = 'RK4';
% RK4 discrete system = RK4 function with function handle (@dynamics) of dynamics function
rk4.f_discrete = @(X,U) RK4(X, U, h, @(X,U) dynamics(X,U,param));
rk4.X = X0;
rk4.U = U;

for k = 1:length(t) - 1
    X_d = dynamics(rk4.X(:,k), U, param);       % calculate accelerations (for nu_d)
    % frequency of controller is slower than simulation
    % only recalculate control input every 4th iteration
    % this corresponds to a 50 Hz controller
    if (mod(k,1) == 0)
        U = trajectory_controller( rk4.X(1:3,k), rk4.X(4:6,k), X_d(4:6), ...
                                   pd(:,k), pd_d(:,k), pd_dd(:,k), pd_ddd(:,k), ...
                                   ke, kphi, kz, delta, param);
    end
    
    % calculate the actual control input from forward force and steering torque
    Uactual(1,:) = 0.5 * (U(1) + U(2) / param(7)) / param(6);
    Uactual(2,:) = 0.5 * (U(1) - U(2) / param(7)) / param(6);
    rk4.U(:, k) = Uactual;

    % saturation
    Uactual(1) = min(max(Uactual(1), 0), 4.5);
    Uactual(2) = min(max(Uactual(2), 0), 4.5);
    rk4.U(:, k) = Uactual;

    rk4.X(:,k+1) = rk4.f_discrete(rk4.X(:,k), Uactual);
end


% visualize results
%visualize(t, rk4.X, 10)

% x-y plot: actual and reference trajectory
figure('DefaultAxesFontSize', 16)
hold on
plot(rk4.X(1, :), rk4.X(2, :), 'b')
plot(pd(1,:), pd(2,:), 'r')
plot(rk4.X(1,1), rk4.X(2,1), 'ob', 'MarkerFaceColor','b','MarkerSize', 10)
plot(pd(1,1), pd(2,1), 'or', 'MarkerFaceColor','r','MarkerSize', 10)
set(gca, 'YDir','reverse')      % Flip direction of y-axis to match coordinate system
grid
axis equal
xlabel('x [m]')
ylabel('y [m]')
legend('actual trajectory','reference trajectory')

% x and y axis separately
figure('DefaultAxesFontSize', 16)
subplot(2,1,1)
hold on
plot(t, rk4.X(1,:))
plot(t, pd(1,:))
ylabel('$x$ [m]')
xlabel('time [s]')
legend('actual trajectory','reference trajectory')
grid
subplot(2,1,2)
hold on
plot(t, rk4.X(2,:))
plot(t, pd(2,:))
ylabel('$y$ [m]')
xlabel('time [s]')
legend('actual trajectory','reference trajectory')
grid

% input u_L and u_R
figure('DefaultAxesFontSize', 16)
subplot(2,1,1)
plot(t(1:end-1), rk4.U(1,:))
ylabel('$u_L$ [-]')
xlabel('time [s]')
grid
subplot(2,1,2)
plot(t(1:end-1), rk4.U(2,:))
ylabel('$u_R$ [-]')
xlabel('time [s]')
grid