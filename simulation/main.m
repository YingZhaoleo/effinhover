%% Simulation of hovercraft 
%  Based on surface ship model by Thor I. Fossen, Guidance and Control of Ocean Vehicles

clear 
close all

%% simulation settings
h = 0.005;            % Sample time
tmax = 10;          % Simulation time   
t = 0:h:tmax;       % Sample times

%% desired trajectory
switch (5)
    case 1
        % circle
        R = 2;  % radius of circle
        pd = R * [cos(2*pi*0.1*t); sin(2*pi*0.1*t)];
        pd_d = R*2*pi*0.1 * [-sin(2*pi*0.1*t); cos(2*pi*0.1*t)];
        pd_dd = R*(2*pi*0.1)^2 * [-cos(2*pi*0.1*t); -sin(2*pi*0.1*t)];
        pd_ddd = R*(2*pi*0.1)^3 * [sin(2*pi*0.1*t); -cos(2*pi*0.1*t)];
    case 2
        % cosine
        pd = [cos(2*pi*0.05*t); t/3];
        pd_d = [-2*pi*0.05*sin(2*pi*0.05*t); ones(size(t))/3];
        pd_dd = [(2*pi*0.05)^2 * cos(2*pi*0.05*t); zeros(size(t))];
        pd_ddd = [-(2*pi*0.05)^3 * sin(2*pi*0.05*t); zeros(size(t))];    
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
        pd_d = [-k*sin(k*t).*cos(t) - cos(k*t).*sin(t); -k*sin(k*t).*sin(t) + cos(k*t).*cos(t)];
        pd_dd = diff(pd_d,1,2)./h;
        pd_dd1 = [pd_dd, [0;0]];
        pd_dd = [ 2*k*sin(t).*sin(k*t) - (k^2 + 1)*cos(t).*cos(k*t); ...
                 -(k^2 + 1)*sin(t).*cos(k*t) - 2*k*cos(t).*sin(k*t)];
    case 5
        % straight line
        p0 = [-2; -2];
        p1 = [3; 2];
        pd = p0.*(1 - t/10) + p1.*t/10;
        pd_d = ones(size(pd)).*(p1 - p0)/10;
        pd_dd = zeros(size(pd));
        pd_ddd = zeros(size(pd));
end

% set some of the temporal derivatives of trajectory to zero
%pd_d = zeros(size(pd));
%pd_dd = zeros(size(pd));
%pd_ddd = zeros(size(pd));

%% controller parameters
ke = 0.005;                     % convergence rate to the trajectory?
kphi = 0.5 * eye(2);        % seems to influence rotation rate a lot
kz = 0.00005;
delta = 0.1*[0.1; 0.1]; 
% no idea what the fuck delta does but it cannot be neither 
% too small nor too big -> hovercraft starts rotating

% max possible input to system
u1_max = 0.16;
u2_max = 10 * 0.16 * 0.0325;    % this should be even lower?

U = [0; 0];                 % initial controller input

%% simulate system using RK4

% initial state vector

X0 = zeros(6, 1);
X0(1:2) = [-3, -1]
%X0(1:2) = pd(:, 1);     % set hovercraft at start of trajectory
%X0(4:5) = pd_d(:, 1);

rk4.name = 'RK4';
% RK4 discrete system = RK4 function with function handle (@dynamics) of dynamics function
rk4.f_discrete = @(X,U) RK4(X, U, h, @dynamics);
rk4.X = X0;
rk4.U = U;

for k = 1:length(t) - 1
    X_d = dynamics(rk4.X(:,k), U, 0);       % calculate accelerations (for nu_d)
    X_d = X_d * 0;
    % frequency of controller is slower than simulation
    % only recalculate control input every nth iteration
    if (mod(k,5) == 0)
        U = trajectory_controller( rk4.X(1:3,k), rk4.X(4:6,k), X_d(4:6), ...
                                   pd(:,k), pd_d(:,k), pd_dd(:,k), pd_ddd(:,k), ...
                                   ke, kphi, kz, delta);
        % saturation
        %U(1) = min(max(U(1), 0), u1_max);   % cannot go backwards 
        %U(2) = min(max(U(2), -u2_max), u2_max); 
    end
    Uactual(1,:) = 0.5 * (U(1) + U(2)/0.0325) / 0.08;
    Uactual(2,:) = 0.5 * (U(1) - U(2)/0.0325) / 0.08;
    Uactual(1) = min(max(Uactual(1), 0), 1);
    Uactual(2) = min(max(Uactual(2), 0), 1);
    %Uactual = U;
    rk4.U(:, k) = U;
    rk4.X(:,k+1) = rk4.f_discrete(rk4.X(:,k), Uactual);
end


% visualize results
visualize(t, rk4.X, 10)

% actual and reference trajectory
figure
hold on
plot(rk4.X(1, :), rk4.X(2, :), 'b')
plot(pd(1,:), pd(2,:), 'r')
set(gca, 'YDir','reverse')      % Flip direction of y-axis to match coordinate system
axis equal
legend('actual trajectory','reference trajectory')

figure
subplot(2,1,1)
plot(t(1:end-1), rk4.U(1,:))
ylabel('u_1 [N]')
xlabel('time [s]')
subplot(2,1,2)
plot(t(1:end-1), rk4.U(2,:))
ylabel('u_2 [Nm]')
xlabel('time [s]')

% % for identification tests
% dataId.name = 'Identification data';
% dataId.t = t(1:5:end);
% dataId.nu = rk4.X(4:6, 1:5:end);
% dataId.U = U(:,1:5:end);
% dataId.h = h;
% dataId.tmax = tmax;
% dataId.nu0 = X0(4:6);
% save('../identification/dataID.mat', 'dataId')