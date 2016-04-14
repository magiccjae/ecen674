clear; close all; clc;
P.gravity = 9.81;
   
%physical parameters of airframe
P.mass = 13.5;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = 0.1204;

P.r = P.Jx*P.Jz-P.Jxz^2;
P.r1 = P.Jxz*(P.Jx-P.Jy+P.Jz)/P.r;
P.r2 = P.Jz*(P.Jz-P.Jy)+P.Jxz^2;
P.r3 = P.Jz/P.r;
P.r4 = P.Jxz/P.r;
P.r5 = (P.Jz-P.Jx)/P.Jy;
P.r6 = P.Jxz/P.Jy;
P.r7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/P.r;
P.r8 = P.Jx/P.r;


% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = 0;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

