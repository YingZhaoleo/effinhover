function U = trajectory_controller_sf(X_d,xr,yr,psir,ur,vr,rr,u1r,u2r);
%TRAJECTORY_CONTROLLER_SF position tracking control of underactuated hovercraft.
%   Reference paper: "Tracking Control of an Underactuated Ship" by Erjen
%   Lefeber Kristin Pattersen and Henk Nijmeijer.

%   Inputs:
%       X_d : general position and velocity vector (x, y, psi, u, v,r ), 6 x 1 vector
%       xr : x reference trajectory
%       yr : y reference trajectory
%       psir : psi reference yaw
%       ur : reference surge
%       vr : reference sway
%       rr : reference yaw rate
%       u1r : reference term in thrust control input
%       u2r : reference term in yaw control input
%
%   Output:
%       U : controller output (u1, u2), 2 x 1 vector
%       u1 : thrust control input
%       u2 : yaw control input

% Decompose state and velocity vector
    x = X_d(1);
    y = X_d(2);
    psi = X_d(3);
    u = X_d(4);
    v = X_d(5);
    r = X_d(6);
   
% Velocity and yaw rate errors 
    ue = u - ur;
    ve = v - vr;
    re = r - rr;
    
    z1 = x*cos(psi) + y*sin(psi);
    z1r = xr*cos(psir) + yr*sin(psir);
    
    z2 = -x*sin(psi) + y*cos(psi);
    z2r = -xr*sin(psir) + yr*cos(psir);

% Position and angle errors    
    z1e = z1 - z1r;
    z2e = z2 - z2r;
    z3e = psi - psir;

% Compute control inputs, preassigned gains
    U(1) = u1r - 10.28*ue + 9.2*ve - 4.44*z1e + 2.74*z2e;
    U(2) = u2r - 9.02*re - 6.74*z3e;
    
end