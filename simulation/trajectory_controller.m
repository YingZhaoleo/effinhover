function u = trajectory_controller( eta, nu, nu_d, pd, pd_d, pd_dd, pd_ddd, ke, kphi, kz, delta)
%TRAJECTORY_CONTROLLER  position tracking control of underactuated hovercraft.
%   Reference paper: "Position Tracking for a Nonlinear Underactuated Hovercraft: 
%   Controller Design and Experimental Results" by Antonio Pedro Aguiar,
%   Lars Cremean and Joao Pedro Hespanha.
%
%   Inputs:
%       eta : pose (x, y, psi), 3 x 1 vector
%       nu : velocity (u, v, r), 3 x 1 vector
%       nu_d: acceleration (du/dt, dv/dt, dr/dt), 3 x 1 vector
%       pd : reference trajectory (x_d(t), y_d(t)), 2 x 1 vector
%       pd_d, pd_dd, pd_ddd : 1st, 2nd and 3rd temporal derivative of pd, 
%                             each 2 x 1 vector
%       ke : controller gain, real number
%       kphi : controller gain, 2 x 2 matrix
%       kz : controller gain, real number
%       delta : controller parameter, 2 x 1 vector
%
%   Output:
%       u : controller output (u1, u2), 2 x 1 vector, 
%           u1 : forward pushing force
%           u2 : steering torque about vertical axis


% parameters of the hovercraft
m = 0.59;       % mass
Iz = 0.106;     % moment of inertia around z axis

dv = 0.3;       % linear damping
dr = 0.5E-2;    % rotational damping

Dv = diag([dv, dv]);
Rpsi = [cos(eta(3)), -sin(eta(3)); sin(eta(3)), cos(eta(3))];
Sr = nu(3) * [0, -1; 1, 0];
B = [1, m * delta(2); 0, m * delta(1)];
Bd = B(:,2);

e = Rpsi'*(eta(1:2) - pd);
e_d = -Sr*e + nu(1:2) - Rpsi'*pd_d;

z1 = nu(1:2) - Rpsi'*pd_d + ke/m*e;
z1_d = nu_d(1:2) + Sr*Rpsi'*pd_d - Rpsi'*pd_dd + ke/m*e_d;

phi = z1 - delta;
phi_d = z1_d;

h = -Dv*Rpsi'*pd_d + ke/m*Dv*e - m*Rpsi'*pd_dd + ke * z1 - ke*ke/m*e;
h_d = -Dv*( -Sr*Rpsi'*pd_d + Rpsi'*pd_dd) + ke/m*Dv*e_d ...
      -m*( -Sr*Rpsi'*pd_dd + Rpsi'*pd_ddd) + ke*z1_d - ke*ke/m*e_d;

alpha = -B\( h - Dv*delta + e/m + kphi*phi/m);
alpha_d = -B\(h_d + e_d/m + kphi*phi_d/m);

z2 = nu(3) - alpha(2);

u(1,1) = alpha(1);
u(2,1) = -m*Bd'*phi + dr*alpha(1) + Iz*alpha_d(2) - kz*z2; 

end

