function [U ue ve re z1e z2e z3e] = trajectory_controller_sf(X_d,xr,yr,psir,ur,vr,rr,u1r,u2r);
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
    
    z1 = x*cos(2*pi*psi) + y*sin(2*pi*psi);
    z1r = xr*cos(2*pi*psir) + yr*sin(2*pi*psir);
    
    z2 = -x*sin(2*pi*psi) + y*cos(2*pi*psi);
    z2r = -xr*sin(2*pi*psir) + yr*cos(2*pi*psir);

% Position and angle errors    
    z1e = z1 - z1r; %x - xr;
    z2e = z2 - z2r; %y - yr;
    z3e = psi - psir;

% Compute control inputs, preassigned gains (open loop by commenting after
% u1r, u2r)
    U(1) = u1r; % - 10.28*ue + 9.2*ve - 4.44*z1e + 2.74*z2e;
    U(2) = u2r; % - 9.02*re - 6.74*z3e;
    
end