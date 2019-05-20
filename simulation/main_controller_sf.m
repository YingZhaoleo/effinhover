%% Simulation of hovercraft 
%  Based on surface ship model by Thor I. Fossen, Guidance and Control of Ocean Vehicles

clear 
close all
clc

%% Simulation settings

h = 0.005;          % Sample time
tmax = 20;          % Simulation time   
t = 0:h:tmax;       % Sample times

%% Desired trajectory (circle)

R = 1;              % Radius of circle
pd = R * [cos(2*pi*0.1*t); sin(2*pi*0.1*t)];
pd_d = R*2*pi*0.1 * [-sin(2*pi*0.1*t); cos(2*pi*0.1*t)];

xr = pd(1,:);
yr = pd(2,:);
psir = 0.05*t;

xr_d = pd_d(1,:);
yr_d = pd_d(2,:);
rr = 0.05;
psir_d = rr*ones(size(psir));

ur = 0.05;
rr = 0.05;

% set some of the temporal derivatives of trajectory to zero
%pd_d = zeros(size(pd));

%% Controller parameters

m = 0.0583;         % mass
Iz = 0.00013;       % moment of inertia around z axis

dv = 1;             % linear damping
dr = 0;             % rotational damping

Dv = diag([dv, dv, dr]);

M = [m, 0, 0;
     0, m, 0;
     0, 0, Iz];
 
% Max possible input to system
u1_max = 0.16;
u2_max = 10 * 0.16 * 0.0325;    % this should be even lower?

U = [0; 0];                     % initial controller input

c1 = 1;
c2 = 1;
k1 = 0.1 + Dv(2,2) - Dv(1,1);
k4 = 0.1;
k3 = 0.05;
k2 = (M(2,2)*k4*(k4+k1+Dv(1,1)-Dv(2,2)))/(Dv(2,2)*k4 + M(1,1)*k3);
k5 = 0.05;
k6 = 1;

%% Simulate system using RK4

% initial state vector
 X0 = zeros(6, 1);
% X0(1:2) = pd(:, 1);
% X0(4:5) = pd_d(:, 1);
% X0 = [4.75, 3.5, pi, 0.05, 0, 0.05]'; % Parameters used in paper

rk4.name = 'RK4';
rk4.f_discrete = @(X,U) RK4(X, U, h, @dynamics);
rk4.X = X0;
rk4.U = U;

for k = 1:length(t) - 1
    X_d = dynamics(rk4.X(:,k), U, 0);

    if (mod(k,1) == 0)
        z = rk4.X(1:3,k);           % is same as eta, but used z to conform with paper
        nu = rk4.X(4:6,k);
        
        % Determined by rearranging xr_d 
        vr = (ur*cos(psir(1,k))-xr_d(1,k)) / sin(psir(1,k));
        
        % Velocity errors, will be redefined as matrix if confirmed working
        u_e = nu(1) - ur;
        v_e = nu(2) - vr;
        r_e = nu(3) - rr;
        
        % Position errors, will be redefined as matrix if confirmed working
        z1_e = z(1) - xr(k);
        z2_e = z(2) - yr(k);
        z3_e = z(3) - psir(k);
        
        % Defined as nu in the paper (equation 12), called nu_c here to
        % avoid confusion with nu vector
        % nu_c = -c1 * r_e - c2*z3_e;
        
        % Preliminary feedback
%        U(2) = u2_r - (M(1,1)-M(2,2)) * (nu(1)*nu(2)-ur*vr) + Dv(3,3)*r_e + M(3,3)*nu_c; % Proposition 1
        u1_r = M(1,1) * ((-M(2,2)/M(1,1))*vr*rr + (Dv(1,1)/M(1,1))*ur);
        u2_r = M(3,3) * ( -(M(1,1)-M(2,2))/M(3,3) * ur*vr + (Dv(3,3)/M(3,3))*rr );
        
        U(2) = u2_r - (M(1,1)-M(2,2)) * (u_e*v_e+vr*u_e+ur*v_e) + k5*r_e - k6*z3_e;
        
        u_ed = v_e*rr - (Dv(1,1)/M(1,1))*u_e + (1/M(1,1))*(U(1) - u1_r);
        v_ed = -(M(1,1)/M(2,2))*u_e*rr - (Dv(2,2)/M(2,2))*v_e;
        
        z1_ed = u_e + z2_e*rr;
        z2_ed = v_e - z1_e*rr;
        z3_ed = r_e;
        
        U(1) = u1_r - k1*u_e + k2*rr*v_e - k3*z1_e + k4*rr*z2_e;
        
        % saturation
        U(1) = min(max(U(1), 0), u1_max);
        U(2) = min(max(U(2), -u2_max), u2_max); 
   end
    
    rk4.U(:, k) = U;
    rk4.X(:,k+1) = rk4.f_discrete(rk4.X(:,k), U);
 
end

%%
%  for k = 1:length(t) - 1
%      X_d = dynamics(rk4.X(:,k), U, 0);
%  
%      if (mod(k,1) == 0)
%          U = trajectory_controller_ss(rk4.X(1:3,k),rk4.X(4:6,k),xr(k),yr(k),...
%                 psir(k),xr_d(k),yr_d(k),ur,rr,rk4.U(1),k1,k2,k3,k4,k5,k6);
%      end
%      
%      rk4.U(:, k) = U;
%      rk4.X(:,k+1) = rk4.f_discrete(rk4.X(:,k), U);
%   
%  end
%%
% visualize results
visualize(t, rk4.X, 20)
%%
% actual and reference trajectory
figure
hold on
plot(rk4.X(1, :), rk4.X(2, :), 'b')
plot(pd(1,:), pd(2,:), 'r')
set(gca, 'YDir','reverse')      % Flip direction of y-axis to match coordinate system
axis equal
legend('actual trajectory','reference trajectory')

figure
subplot(2,1,1)
plot(t(1:end-1), rk4.U(1,:))
ylabel('u_1 [N]')
xlabel('time [s]')
subplot(2,1,2)
plot(t(1:end-1), rk4.U(2,:))
ylabel('u_2 [Nm]')
xlabel('time [s]')