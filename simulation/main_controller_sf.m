%% Simulation of hovercraft 
%  Based on surface ship model by Thor I. Fossen, Guidance and Control of Ocean Vehicles

clear 
close all
clc

%% Simulation settings
h = 0.01;           % Sample time
tmax = 200;         % Simulation time   
t = 0:h:tmax;       % Time vector

% Remember to set same parameters in dynamics function
m = 17.6;
Iz = 1.98;

% Neglect damping effects
d11 = 0;
d22 = 0;
d33 = 0;

% Desired trajectory: circle
R = 1;      % Radius of circle
w = 0.05; 

% convert all to rad, all angles should be to -pi to pi, z3_err scaled
% (multiply by 2pi, use wrapTopi command)
psir = w*t;

xr = R*cos(2*pi*psir);
yr = R*sin(2*pi*psir);

% Trajectory derivatives
xr_d = R*2*pi*w * (-sin(2*pi*psir));
xr_dd = R*(2*pi*w)^2 *cos(2*pi*psir);

yr_d = R*2*pi*w * cos(2*pi*psir);
yr_dd = R*(2*pi*w)^2 * sin(2*pi*psir);

% Desired velocities and yaw rate with derivatives
ur = xr_d.*cos(2*pi*psir) + yr_d.*sin(2*pi*psir); %ur = 0.05;
vr = -xr_d.*sin(2*pi*psir) + yr_d.*cos(2*pi*psir);

rr = -0.05;
ur_d = 0;
rr_d = 0;

% Reference sway, obtained by rearranging xr_d (initial attempt, not
% working)
% vr = (1./sin(2*pi*psir)) .* (ur*cos(2*pi*psir)-xr_d);
% vr(1) = 0;        % Override division by 0 in first term

% Controller reference terms (suggestion but not working)
%u1r = (xr_dd.*cos(2*pi*psir) + xr_d.*2*pi*rr.*sin(2*pi*psir) + yr_dd.*sin(2*pi*psir)...
    %+ yr_d*2*pi*rr.*cos(2*pi*psir) - vr*rr + d11*ur) * m;

u1r = -m*vr*rr + d11*ur;
u2r = -(m-m).*ur.*vr + d33*rr;

% Max input to system
u1_max = 0.9;
u2_max = 0.9;
%% Simulate system

% Initial control input
U = [0; 0];

% Initial state vector
X0 = zeros(6, 1);
X0(1) = -1;%4.75;
X0(2) = yr(1);%3.5;
X0(3) = pi;
X0(4) = ur(1);
X0(5) = vr(1);
X0(6) = rr;

rk4.name = 'RK4';
rk4.f_discrete = @(X,U) RK4(X, U, h, @dynamics);
rk4.X = X0;
rk4.U = U;

for k = 1:length(t) - 1
    X_d = dynamics(rk4.X(:,k), U, 0);
   
    if (mod(k,1) == 0)
        [U ue ve re z1e z2e z3e] = trajectory_controller_sf(X_d,xr(k),yr(k),psir(k),ur(k),...
            vr(k),rr,u1r(k),u2r(k));
        U(1) = min(max(U(1), 0), u1_max); 
        U(2) = min(max(U(2), -u2_max), u2_max);
        U = [U(1);U(2)];
    end
    
    z1_err(k) = z1e;
    z2_err(k) = z2e;
    z3_err(k) = z3e;
    u_err(k) = ue;
    v_err(k) = ve;
    r_err(k) = re;
    rk4.U(:, k) = U;
    rk4.X(:,k+1) = rk4.f_discrete(rk4.X(:,k), U);
end

%% Visualise and plot results
%visualize(t, rk4.X, 20)
 
%% Actual and reference trajectories
figure
hold on
plot(rk4.X(1, :), rk4.X(2, :), 'b')
plot(xr, yr, 'r')
set(gca, 'YDir','reverse')
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
%%
%     x = X_d(1);
%     y = X_d(2);
%     psi = X_d(3);
%     u = X_d(4);
%     v = X_d(5);
%     r = X_d(6);
%     
%     ue = u - ur;
%     ve = v - vr(k);
%     re = r - rr;
%     
%     z1 = x*cos(psi) + y*sin(psi);
%     z1r = xr(k)*cos(2*pi*psir(k)) + yr(k)*sin(2*pi*psir(k));
%     
%     z2 = -x*sin(psi) + y*cos(psi);
%     z2r = -xr(k)*sin(2*pi*psir(k)) + yr(k)*cos(2*pi*psir(k));
%     
%     z1e = z1 - z1r;
%     z2e = z2 - z2r;
%     z3e = psi - 2*pi*psir(k);
%     
%     U(1) = u1r(k) - 10.28*ue + 9.2*ve - 4.44*z1e + 2.74*z2e;
%     U(2) = u2r(k) - 9.02*re - 6.74*z3e;