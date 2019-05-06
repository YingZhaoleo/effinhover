%% Hovercraft parameter identification - Experiments
%  main program for fitting parameters of ODE hovercraft model to data
%  objective function is defined in objFun.m
%  dynamic model is defined in dynamicsId.m
%

clear
clc
close all

% path to identification data
data_id_path = '../../hovercraft-ros/data_id/';

poses = dir([data_id_path, 'pose_2019-04-16*.csv']);
controls = dir([data_id_path, 'controls_2019-04-16*.csv']);


%poses = dir([data_id_path, 'pose_2019-04-16-12-29-28.csv']);

%controls = dir([data_id_path, 'controls_2019-04-16-12-29-28.csv']);

n_files = length(poses);

theta_estim = zeros(n_files, 5);    % save parameters

% load data for every run
for n = 1:6
    pose = readtable([data_id_path, poses(n).name]);
    ctrl = readtable([data_id_path, controls(n).name]);
    
    t_pose = pose.field_header_stamp';
    x = pose.field_point_x';
    y = pose.field_point_y';
    psi = pose.field_point_z';
    psi = unwrap(psi);                   % eliminate jumps and make psi continuous
    %psi = medfilt1(psi, 10);             % eliminate spikes 
    
    t_ctrl = ctrl.field_header_stamp';
    u_L = ctrl.field_point_x';
    u_R = ctrl.field_point_y';
    u = [u_L; u_R];
    
    % find start time and convert from nanoseconds to seconds
    t_start = min(t_pose(1), t_ctrl(1));
    t_pose = (t_pose - t_start)/1e9;
    t_ctrl = (t_ctrl - t_start)/1e9;
    t_ctrl(1) = 0;
    
    % downsample measurements
    t_skip = 10;
    t_start = 5; % timedelay
    t_end = 500; %length(t_pose);
    t_pose = t_pose(t_start:t_skip:t_end);
    x = x(t_start:t_skip:t_end);
    y = y(t_start:t_skip:t_end);
    psi = psi(t_start:t_skip:t_end);
        
    eta = [x; y; psi];

    %visualize(t_pose, eta, 1);

    
    % velocities in world frame
    vel_world = diff(eta, 1, 2)./diff(t_pose);
    
    % calculate velocities (u, v, r) in body frame
    nu = zeros(size(vel_world));
    for k = 1:(length(t_pose) - 1) 
        R_z_psi_T = [cos(eta(3, k)), sin(eta(3, k)), 0;
                   -sin(eta(3, k)),  cos(eta(3, k)), 0;
                         0     ,       0     , 1];
        nu(:,k) = R_z_psi_T * vel_world(:, k);          
    end
    
%     figure
%     plot(nu')
%     nu = medfilt1(nu, 3, [], 2);   % filter nu to eliminate spikes
%     hold on
%     plot(nu')
%     
    dataId.t_nu = t_pose(1:end-1);
    dataId.nu = nu;
    dataId.t_U = t_ctrl;
    dataId.U = u;
    dataId.h = 0.005;
    
    theta0 = [0.08, 0.016, 1, 1, 0.0005];
    
    [thetamin, objmin] = paramId(dataId, theta0)
    theta_estim(n, :) = thetamin;
    
    
    % Plot simulated trajectory with estimated parameters
    
    % Simulation setup
    h = 0.005;           % sampling time
    tmax = t_pose(end);     % simulation time   
    t = 0:h:tmax;         % simulation time 
    X0 = [eta(:,1); nu(:,1)];       % initial condition

    % Upsampling input signal for simulation with nearest interpolation
    Usim = interp1(t_ctrl, u', t, 'previous', 'extrap');
    Usim = Usim';
    
    
    dynamics =@(X, U) dynamicsParam(X, U, thetamin);

    % RK4 discrete system = RK4 function with function handle (@dynamics) of dynamics function
    sim.f_discrete = @(X,U) RK4(X, U, h, dynamics);
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
    plot(t_pose(1:end-1), nu(1,:));
    legend('sim', 'meas');
    xlabel('t [s]')
    ylabel('u [m/s]')
    subplot(3,1,2)
    hold on
    plot(t, sim.X(5,:));
    plot(t_pose(1:end-1), nu(2,:));
    legend('sim', 'meas');
    xlabel('t [s]')
    ylabel('v [m/s]')
    subplot(3,1,3)
    hold on
    plot(t, sim.X(6,:));
    plot(t_pose(1:end-1), nu(3,:));
    legend('sim', 'meas');
    xlabel('t [s]')
    ylabel('r [rad/s]')
    
    drawnow
end
