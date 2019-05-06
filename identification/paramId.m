function [thetamin, objmin] = paramId(dataId, theta0)
%PARAMID Identify parameters of ODE describing hovercraft dynamics
%
% Inputs:
%   theta0 : Vector with initial parameter guess
%   dataId : Structure with identification data containing the following
%               dataId.name : name of data
%               data.t      : time of samples
%               dataId.nu   : state (u,v,r) 
%               dataId.U    : input (u1, u2)
%               dataId.h    : sampling time
%               dataId.tmax : simulation / experiment duration
%               dataId.nu0  : initial state
% Outputs:
%   thetamin : Parameter vector that minimises loss 
%   objmin   : Minimal value of loss


obj =@(theta) objFun(theta, dataId);    % Objective function for minimization
lb = [-1000, 0, 0, 0, 0];               % Lower bound on parameters
ub = 5*ones(1,5);                       % Upper bound on parameters
opt = optimoptions('fmincon', 'Algorithm','interior-point', 'Display', 'iter');    % Optimization settings
%opt = optimoptions('fmincon', 'Algorithm','sqp', 'Display', 'iter');    % Optimization settings

% Minimization
tic
[thetamin, objmin, exitflag, output] = fmincon(obj,theta0,[],[],[],[],lb,ub,[], opt);
toc


end

