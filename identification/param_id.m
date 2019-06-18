function [thetamin, objmin] = param_id(data, theta0)
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


obj = @(theta) objective_fun(theta, data);    % objective function for minimization
lb = [0, 0, 0, 0, -1000];                  % lower bound on parameters
ub = 10*ones(1,5);                       % upper bound on parameters
%opt = optimoptions('fmincon', 'Algorithm','interior-point', 'Display', 'iter');    % optimization settings
opt = optimoptions('fmincon', 'Algorithm','sqp', 'Display', 'iter');    % optimization settings

% minimization
tic
[thetamin, objmin, exitflag, output] = fmincon(obj,theta0,[],[],[],[],lb,ub,[], opt);
toc

end

