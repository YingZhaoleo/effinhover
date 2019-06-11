%% Hovercraft parameter identification - Experiments
%  main program for fitting parameters of ODE hovercraft model to data
%  objective function is defined in objFun.m
%

clear
clc
close all

% import functions from simulation folder
addpath('../simulation');


% path to identification data
data_id_path = 'identification_data/set01/';

eta_file = dir([data_id_path, 'eta*.csv']);
nu_file = dir([data_id_path, 'nu*.csv']);
ctrl_file = dir([data_id_path, 'controls*.csv']);


n_files = length(eta_file);

theta_estim = zeros(n_files, 4);    % save parameters

% load data for every run
for n = 1:n_files
    
    % import data as tables
    eta_read = readtable([data_id_path, eta_file(n).name]);
    nu_read = readtable([data_id_path, nu_file(n).name]);
    ctrl_read = readtable([data_id_path, ctrl_file(n).name]);
    
    t_eta = eta_read.field_header_stamp';   % same time for all imported variables
    t_eta = (t_eta - t_eta(1)) / 1e9;       % set start time to 0 and convert to seconds
    eta = [eta_read.field_point_x, eta_read.field_point_y, eta_read.field_point_z]';
    nu = [nu_read.field_point_x, nu_read.field_point_y, nu_read.field_point_z]';
    ctrl = [ctrl_read.field_point_x, ctrl_read.field_point_y]';
    
    %psi = pose.field_point_z';
    %psi = unwrap(psi);                   % eliminate jumps and make psi continuous
    %psi = medfilt1(psi, 10);             % eliminate spikes 
    
    %nu(2,:) = medfilt1(nu(2,:), 3, [], 2);   % filter nu to eliminate spikes

   
    % downsample measurements
    %t_skip = 10;
    %t_start = 5; % timedelay
    %t_end = 500; %length(t_pose);
    %t_pose = t_pose(t_start:t_skip:t_end);
    %x = x(t_start:t_skip:t_end);
    %y = y(t_start:t_skip:t_end);
    %psi = psi(t_start:t_skip:t_end);
        
    %eta_file = [x; y; psi];

    %visualize(t_eta, eta, 1);

    
%     figure
%     plot(nu')
%     nu = medfilt1(nu, 3, [], 2);   % filter nu to eliminate spikes
%     hold on
%     plot(nu')
%     
    dataId.t_nu = t_eta;
    dataId.nu = nu;
    dataId.t_U = t_eta;
    dataId.U = ctrl;
    dataId.h = 0.005;
    
    theta0 = [0.8, 0.016, 0.05, 0.005];
    
    [thetamin, objmin] = param_id(dataId, theta0)
    theta_estim(n, :) = thetamin;
    
    
    % Plot simulated trajectory with estimated parameters
    
    plot_stuff = true;
    
    if (plot_stuff)
        % Simulation setup
        h = 0.005;           % sampling time
        tmax = t_eta(end);     % simulation time   
        t = 0:h:tmax;         % simulation time 
        X0 = [eta(:,1); nu(:,1)];       % initial condition

        % Upsampling input signal for simulation with nearest interpolation
        Usim = interp1(t_eta, ctrl', t, 'previous', 'extrap');
        Usim = Usim';


        dynamics_param =@(X, U) dynamics(X, U, [0.058, thetamin, 0.0325]);

        % RK4 discrete system = RK4 function with function handle (@dynamics) of dynamics function
        sim.f_discrete = @(X,U) RK4(X, U, h, dynamics_param);
        sim.X = X0;

        for k = 1:length(t) - 1
          sim.X(:,k+1) = sim.f_discrete(sim.X(:,k), Usim(:,k));
        end


        %visualize(t(:), sim.X(:,:), 1);


        figure
        hold on
        plot(sim.X(1,:), sim.X(2,:));
        plot(eta(1,:), eta(2,:));
        legend('sim', 'meas');
        set(gca, 'YDir','reverse')      % Flip direction of y-axis to match coordinate system
        axis equal
        xlabel('x [m]')
        ylabel('y [m]')

        figure
        subplot(3,1,1)
        hold on
        plot(t, sim.X(4,:));
        plot(t_eta, nu(1,:));
        legend('sim', 'meas');
        xlabel('t [s]')
        ylabel('u [m/s]')
        subplot(3,1,2)
        hold on
        plot(t, sim.X(5,:));
        plot(t_eta, nu(2,:));
        legend('sim', 'meas');
        xlabel('t [s]')
        ylabel('v [m/s]')
        subplot(3,1,3)
        hold on
        plot(t, sim.X(6,:));
        plot(t_eta, nu(3,:));
        legend('sim', 'meas');
        xlabel('t [s]')
        ylabel('r [rad/s]')

        drawnow
    end
end
