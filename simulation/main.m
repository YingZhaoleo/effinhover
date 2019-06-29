%% Simulation of hovercraft 
%  Based on surface ship model by Thor I. Fossen, Guidance and Control of Ocean Vehicles

clear 
%close all

%% simulation settings
h = 0.005;          % sample time
tmax = 50;          % simulation time   
t = 0:h:tmax;       % time vector

%% hovercraft parameters

hovercraft = 'TinyWhoover';     % TinyWhoover or MVWT (reference paper)

switch (hovercraft)
    case 'TinyWhoover'
        m = 0.0583;       % mass
        Iz = 8.8e-5;      % moment of inertia around z axis
        Xu = 2.71e-2;     % linear damping
        Nr = 4.62e-5;     % yaw damping
        k_th = 6.2e-2;    % thrust conversion coefficient
        l = 0.0325;       % lateral offset of thruster from center line
        u_max = 1.0;      % maximum input
    case 'MVWT'
        m = 5.15;        % mass
        Iz = 0.047;      % moment of inertia around z axis
        Xu = 4.5;        % linear damping
        Nr = 0.41;       % yaw damping
        k_th = 1;        % thrust conversion coefficient
        l = 0.123;       % lateral offset of thruster from center line
        u_max = 4.5;     % maximum input
end

param = [m, Iz, Xu, Xu, Nr, k_th, l];

%% desired trajectory
switch (1)
    case 1
        % circle
        R = 2.0;                  % radius of circle
        w = 2.0 * pi / 10.0;    % angular velocity
        w = 0.6;
        pd = R * [cos(w*t); sin(w*t)];
        pd_d = R*w * [-sin(w*t); cos(w*t)];
        pd_dd = R*(w)^2 * [-cos(w*t); -sin(w*t)];
        pd_ddd = R*(w)^3 * [sin(w*t); -cos(w*t)];
    case 2
        % cosine
        w = 2 * pi / 10;
        w = 0.6;
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
        speed = 0.707;        % speed in m/s
        p0 = [-2.5; 2.5];   % starting point
        dir = [1; -1];      % direction vector
        pd = p0 + speed*t.*dir/norm(dir);
        pd_d = speed*dir.*ones(size(t));
        pd_dd = zeros(size(pd));
        pd_ddd = zeros(size(pd));
        
        % stay at the endpoint after reaching it
        idx = find(t > 15, 1);
        pd(:, idx:end) = pd(:, idx).*ones(2, length(t) - idx + 1);
        pd_d(:, idx:end) = 0;
end


%% controller parameters

switch (hovercraft)
    case 'TinyWhoover'
        ke = 4.0;
        kphi = 0.005 * eye(2);
        kz = 0.0003;
        delta = -0.1*[0.1; 0.0]; 
    case 'MVWT'
        ke = 4.0;          
        kphi = 1.5 * eye(2);
        kz = 0.6;
        delta = -0.1*[0.1; 0]; 
end

%% simulate system using RK4

% initial state vector
X0 = zeros(6, 1);
X0(1:2) = [2, 2]

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
    if (mod(k,4) == 0)
        U = trajectory_controller( rk4.X(1:3,k), rk4.X(4:6,k), X_d(4:6), ...
                                   pd(:,k), pd_d(:,k), pd_dd(:,k), pd_ddd(:,k), ...
                                   ke, kphi, kz, delta, param);
    end
    
    % calculate the actual control input from forward force and steering torque
    u_L = 0.5 * (U(1) + U(2) / param(7)) / param(6);
    u_R = 0.5 * (U(1) - U(2) / param(7)) / param(6);
    
    % saturation
    u_L = min(max(u_L(1), 0), u_max);
    u_R = min(max(u_R, 0), u_max);
    
    rk4.U(:, k) = [u_L; u_R];
    rk4.X(:,k+1) = rk4.f_discrete(rk4.X(:,k), [u_L; u_R]);
end


% visualize results
%visualize(t, rk4.X, 20)

% x-y plot: actual and reference trajectory
figure('DefaultAxesFontSize', 16)
hold on
plot(rk4.X(1, :), rk4.X(2, :), 'b', 'LineWidth', 0.75)
plot(pd(1,:), pd(2,:), 'r', 'LineWidth', 0.75)
plot(rk4.X(1,1), rk4.X(2,1), 'ob', 'MarkerFaceColor','b','MarkerSize', 10)
plot(pd(1,1), pd(2,1), 'or', 'MarkerFaceColor','r','MarkerSize', 10)
set(gca, 'YDir','reverse')      % Flip direction of y-axis to match coordinate system
grid
axis equal
xlabel('x [m]')
ylabel('y [m]')
%legend('actual trajectory','reference trajectory')

% x and y axis separately
figure('DefaultAxesFontSize', 16)
subplot(2,1,1)
hold on
plot(t, rk4.X(1,:), 'LineWidth', 0.75)
plot(t, pd(1,:), 'LineWidth', 0.75)
ylabel('$x$ [m]')
xlabel('time [s]')
legend('actual trajectory','reference trajectory')
grid
subplot(2,1,2)
hold on
plot(t, rk4.X(2,:), 'LineWidth', 0.75)
plot(t, pd(2,:), 'LineWidth', 0.75)
ylabel('$y$ [m]')
xlabel('time [s]')
legend('actual trajectory','reference trajectory')
grid

% input u_L and u_R
figure('DefaultAxesFontSize', 16)
subplot(2,1,1)
plot(t(1:end-1), rk4.U(1,:), 'LineWidth', 0.75)
ylabel('$u_L$ [-]')
xlabel('time [s]')
grid
subplot(2,1,2)
plot(t(1:end-1), rk4.U(2,:), 'LineWidth', 0.75)
ylabel('$u_R$ [-]')
xlabel('time [s]')
grid