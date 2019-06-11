function S = objective_fun(theta, data)
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
%
% Output:
%   S : Value of objective function for current parameters theta

% Assign parameters
m = 0.058;
l = 0.0325;
K = theta(1);
Iz = theta(2);
Xu = theta(3);
Nr = theta(4);

% Simulation setup
h = data.h;           % sampling time
tmax = data.t_nu(end);     % simulation time   
t = 0:h:tmax;         % simulation time 
nu0 = data.nu(:,1);       % initial condition

% Upsampling input signal for simulation with nearest interpolation
Ures = interp1(data.t_U, data.U', t, 'previous', 'extrap');

data.U = Ures';

% Wrap dynamicsId function into function that can be used by RK4
dynamics =@(nu, U) dynamics_reduced(nu, U, [m, Iz, Xu, Nr, K, l]);

% Simulate system with current parameter using Euler forward
% Euler discrete system = Euler function with function handle (@dynamicsId) of dynamics function
sim.f_discrete = @(nu,U) RK4(nu, U, h, dynamics);
sim.nu = nu0;
for k = 1:length(t) - 1
    sim.nu(:,k+1) = sim.f_discrete(sim.nu(:,k), data.U(:,k));
end

% Compute predicted states at each (measured) sampling time by
% interpolating the simulation results at the measured sampling times
nuPred = interp1(t, sim.nu', data.t_nu, 'nearest', 'extrap');


% Compute sum of squared difference between model and identification data
%S = sum(sum( (nuPred' - data.nu).^2 ))/length(data.t_nu);

S = sum(vecnorm(nuPred' - data.nu, 2, 2))/length(data.t_nu);

end