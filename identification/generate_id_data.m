%% Generate artificial identification data

clear 
clc
close all

% import functions from simulation folder
addpath('../simulation');

%% parameters of id data
add_noise = true;      % add noise to simulated velocity vector
noise_level = 0.1;      % amplitude of noise
h_out = 0.1;            % sampling time of id data
t_max = 10;             % time duration of id data  

%% simulation settings
h = 0.005;          % simulation time step
t = 0:h:t_max;       % simulation time vector

%% hovercraft parameters
m = 0.0583;       % mass of hovercraft
Iz = 0.00013;     % moment of inertia around z axis
Xu = 0.05;        % linear damping
Nr = 0.0001;     % yaw damping
motor_coeff = 0.08;         % motor signal to thrust conversion coefficient
l = 0.0325;       % lateral offset of thruster from center line

param = [m, Iz, Xu, Xu, Nr, motor_coeff, l];

%% identification input

u_fwd = 0.5;
u_diff = 0.2 * idinput(length(t), 'prbs', [0, 10*h]);

U = u_fwd + [u_diff, - u_diff]';

figure
hold on
plot(t, U(1,:))
plot(t, U(2,:))

%% simulate system using RK4

% initial state vector

X0 = zeros(6, 1);

sim.f_discrete = @(X,U) RK4(X, U, h, @(X,U) dynamics(X,U,param));
sim.X = X0;
sim.U = U;

for k = 1:length(t) - 1
    sim.X(:,k+1) = sim.f_discrete(sim.X(:,k), U(:,k));
end

%% visualize results
%visualize(t, sim.X, 10)

% trajectory
figure
plot(sim.X(1, :), sim.X(2, :), 'b')
set(gca, 'YDir','reverse')      % Flip direction of y-axis to match coordinate system
axis equal

%% save identification data

nu = sim.X(4:6, :);
if add_noise
    nu = nu + noise_level * std(nu, [], 2).*randn(size(nu));
end

step = h_out/h;
data_id.name = 'artificial identification data';
data_id.true_param = [Iz, Xu, Nr, motor_coeff];
data_id.t = t(1:step:end);
data_id.eta = sim.X(1:3, 1:step:end);
data_id.nu = nu(:, 1:step:end);
data_id.U = U(:,1:step:end);
save('../identification/test_data_id.mat', 'data_id')