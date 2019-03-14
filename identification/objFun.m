function S = objFun(theta, data)
% OBJFUN objective function of parameter estimation 
% S = sum of squared differences between model and identification data
%
% Inputs:
%   theta : Vector with parameters to estimate
%   data  : Structure with identification data containing the following
%               dataId.name : name of data
%               dataId.nu   : state (u,v,r) 
%               dataId.U    : input (u1, u2)
%               dataId.h    : sampling time
%               dataId.tmax : simulation / experiment duration
%               dataId.nu0  : initial state
%
% Output:
%   S : Value of objective function for current parameters theta

% Assign parameters
m = 0.59;
Iz = theta(2);
Xu = theta(3);
Yv = theta(4);
Nr = theta(5);
K = theta(1);

% Simulation setup
h = data.h;           % sampling time
tmax = data.tmax;     % simulation time   
t = 0:h:tmax;
nu0 = data.nu0;       % initial condition

% Wrap dynamicsId function into function that can be used by RK4
dynamics =@(nu, U) dynamicsId(nu, U, m, Iz, Xu, Yv, Nr, K);

% % Simulate system with current parameter using RK4
rk4.name = 'RK4';
% RK4 discrete system = RK4 function with function handle (@dynamics) of dynamics function
rk4.f_discrete = @(nu,U) RK4(nu, U, h, dynamics);
rk4.nu = nu0;
for k = 1:length(t) - 1
    rk4.nu(:,k+1) = rk4.f_discrete(rk4.nu(:,k), data.U(:,k));
end

% find predicted values x(tdata)
%xpred = interp1(tsol,xsol(:,1),tdata);

% Compute sum of squared difference between model and identification data
S = sum(sum( (rk4.nu - data.nu).^2));

end