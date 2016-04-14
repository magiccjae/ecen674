clear; close all; clc;

P.Ts = 0.01;
P.u0 = 0;
P.w0 = 0;
P.q0 = 0;
P.theta0 = 0;
P.h0 = 0;

% Pitch attitude hold
P.delta_e_max = 30*pi/180;
P.e_theta_max = 10*pi/180;
a_theta1 = 0.668;
a_theta2 = 1.27;
a_theta3 = -2.08;
P.kp_theta = P.delta_e_max / P.e_theta_max * sign(a_theta3);
omega_theta = sqrt(a_theta2 + P.delta_e_max / P.e_theta_max * abs(a_theta3));
zeta_theta = 0.707;     % design parameter
P.kd_theta = (2 * zeta_theta * omega_theta - a_theta1) / a_theta3;

K_theta_dc = P.kp_theta * a_theta3 / (a_theta2 + P.kp_theta * a_theta3);

% Altitude hold using Pitch
P.Va0 = 830;
W_h = 20;       % design parameter
omega_h = 1 / W_h * omega_theta;
Va = P.Va0;
zeta_h = 1;     % design parameter
P.ki_h = omega_h^2 / (K_theta_dc * Va);
P.kp_h = 2 * zeta_h * omega_h / (K_theta_dc * Va);
P.theta_max = 45*pi/180;
