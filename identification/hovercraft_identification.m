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

% load data for every run
for k = 1: n_files
    pose = readtable([data_id_path, poses(k).name]);
    ctrl = readtable([data_id_path, controls(k).name]);
    
    t_pose = pose.field_header_stamp;
    x = pose.field_point_x;
    y = pose.field_point_y;
    psi = pose.field_point_z;

    t_ctrl = ctrl.field_header_stamp;
    u_r = ctrl.field_point_x;
    u_l = ctrl.field_point_y;
    
    % find start time and convert from nanoseconds to seconds
    t_start = min(t_pose(1), t_ctrl(1));
    t_pose = (t_pose - t_start)/1e9;
    t_ctrl = (t_ctrl - t_start)/1e9;
    
    visualize(t_pose', [x, y, psi-pi/2]', 10);
end
