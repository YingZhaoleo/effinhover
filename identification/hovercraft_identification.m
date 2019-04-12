%% Hovercraft parameter identification - Experiments
%  main program for fitting parameters of ODE hovercraft model to data
%  objective function is defined in objFun.m
%  dynamic model is defined in dynamicsId.m
%

clear
clc

% path to identification data
data_id_path = '../../hovercraft-ros/data_id/';

poses = dir([data_id_path, 'pose*.csv']);
controls = dir([data_id_path, 'controls*.csv']);
n_files = length(poses);

n_files = 1;

% load data for every run
for n = 1:n_files
    pose = readtable([data_id_path, poses(n).name]);
    ctrl = readtable([data_id_path, controls(n).name]);
    
    t_pose = pose.field_header_stamp';
    x = pose.field_point_x';
    y = pose.field_point_y';
    psi = pose.field_point_z' - pi/2;    % align coordinate systems
    
    eta = [x; y; psi];

    t_ctrl = ctrl.field_header_stamp';
    u_r = ctrl.field_point_x';
    u_l = ctrl.field_point_y';
    u = [u_r; u_l];
    
    % find start time and convert from nanoseconds to seconds
    t_start = min(t_pose(1), t_ctrl(1));
    t_pose = (t_pose - t_start)/1e9;
    t_ctrl = (t_ctrl - t_start)/1e9;
    t_ctrl(1) = 0;
    
    %visualize(t_pose, eta, 10);
    
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
    
    dataId.t_nu = t_pose(1:end-1);
    dataId.nu = nu;
    dataId.t_U = t_ctrl;
    dataId.U = u;
    dataId.h = 0.005;
    
    theta0 = [0.001, 0.003, 0.003, 0.003, 0.005];
    
    [thetamin, objmin] = paramId(dataId, theta0)
end
