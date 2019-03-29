%% Hovercraft parameter identification
%  main program for fitting parameters of ODE hovercraft model to data
%  objective function is defined in objFun.m
%  dynamic model is defined in dynamicsId.m
%
%  To obtain new identification data run the main.m function in
%  effinhover/identification and specify any input in the file

clear
close all
load dataID.mat

%% Initial parameters for model

m = 0.59;       % Mass of hovercraft
Iz = 0.106;     % Moment of inertia around z axis

Xu = 0.3;       % Surge damping
Yv = 0.35;      % Sway damping
Nr = 0.5E-2;    % Yaw damping

K = 0.1;          % Motor signal to thrust conversion coefficient

% Initial parameter guess
theta0 = [K, Iz, Xu, Yv, Nr] + 0.3*rand;

% Add some noise to identification data
dataId.nu = dataId.nu + 0.1*(0.5-rand(size(dataId.nu)));

%% Minimization procedure

obj =@(theta) objFun(theta, dataId);    % Objective function for minimization
lb = zeros(1,5);                        % Lower bound on parameters
ub = 5*ones(1,5);                       % Upper bound on parameters
opt = optimoptions('fmincon', 'Algorithm','sqp', 'Display', 'iter');    % Optimization settings

% Minimization
tic
[thetamin, objmin, exitflag, output] = fmincon(obj,theta0,[],[],[],[],lb,ub,[], opt);
toc

disp('Real parameters theta:');
disp([K, Iz, Xu, Yv, Nr]);
disp('Estimated parameters theta:');
disp(thetamin)
disp('Minimum of objective:');
disp(objmin)