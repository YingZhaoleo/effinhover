function S = objFun(theta, data)
% OBJFUN objective function of parameter estimation 
% S = sum of squared differences between model and identification data
%
% Inputs:
%   theta : Vector with parameters to estimate
%   data  : Structure with identification data containing the following
%               dataId.name : name of data
%               data.t      : time of samples
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
K = theta(1);
Iz = theta(2);
Xu = theta(3);
Yv = theta(4);
Nr = theta(5);

% Simulation setup
h = data.h;           % sampling time
tmax = data.tmax;     % simulation time   
t = 0:h:tmax;         % simulation time 
nu0 = data.nu0;       % initial condition

% Upsampling input signal for simulation with nearest interpolation
Ures = interp1(data.t, data.U', t, 'previous');

data.U = Ures';
% Wrap dynamicsId function into function that can be used by RK4
dynamics =@(nu, U) dynamicsId(nu, U, m, Iz, Xu, Yv, Nr, K);

% Simulate system with current parameter using Euler forward
% Euler discrete system = Euler function with function handle (@dynamicsId) of dynamics function
sim.f_discrete = @(nu,U) EM(nu, U, h, dynamics);
sim.nu = nu0;
for k = 1:length(t) - 1
    sim.nu(:,k+1) = sim.f_discrete(sim.nu(:,k), data.U(:,k));
end

% Compute predicted states at each (measured) sampling time by
% interpolating the simulation results at the measured sampling times
nuPred = interp1(t, sim.nu', data.t, 'nearest');

% Compute sum of squared difference between model and identification data
S = sum(sum( (nuPred' - data.nu).^2));

end