%% Speed-to-thrust parameter estimation
%  Data seems linear -> use linear regression

load('thrust_measurement.mat');

% average measurements
u = linspace(0, 1, 11)';
thrust_l = ave.left_g / 1000 * 9.81;
thrust_r = ave.right_g / 1000 * 9.81;

% linear regression: thrust = beta(1)*u 
k_l = u\thrust_l
k_r = u\thrust_r

% quadratic regression: thrust = beta(1)*u + beta(2)*u^2
%beta_l = [u, u.^2]\thrust_l
%beta_r = [u, u.^2]\thrust_r

figure('DefaultAxesFontSize', 16)
hold on
grid
scatter(u, thrust_l, 100, 'r');
scatter(u, thrust_r, 100, 'b');
plot(u, k_l*u, 'r--', 'linewidth', 0.75);
plot(u, k_r*u, 'b--', 'linewidth', 0.75);
legend('measurements left', 'measurements right', ...
       'regression left', 'regression right', ...
       'location', 'NW')
xlabel('input $u_L$, $u_R$ [-]')
ylabel('speed-to-thrust coefficient k [N]')


