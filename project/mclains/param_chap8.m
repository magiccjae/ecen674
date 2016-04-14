P.gravity = 9.81;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 25;%13.5;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;
% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
%P.k_motor = 15;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
%     % HACK: prop has too much power for aerosonde
%     P.C_prop = 0.5;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

% wind parameters
P.wind_n = 3;
P.wind_e = 2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;


% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 35;        % m/s (~85 mph)
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = Inf;         % desired radius (m) - use (+) for right handed orbit, 
h0    = 100;  % initial altitude

% autopilot sample rate
P.Ts = 0.01;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -h0;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

                    %                          (-) for left handed orbit

% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -h0;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);


%----------------------------------------------------
% low level autopilot gains
    P.tau = 5;  % gain on dirty derivative
    P.altitude_take_off_zone = 10;
    P.altitude_hold_zone = 10;
    P.theta_c_max = 30*pi/180; % maximum pitch angle command
    P.climb_out_trottle = 0.25;

% select gains for roll loop
    % get transfer function data for delta_a to phi
    [num,den]=tfdata(T_phi_delta_a,'v');
    a_phi2 = num(3);
    a_phi1 = den(2);
    % maximum possible aileron command
    delta_a_max = 45*pi/180;
    % Roll command when delta_a_max is achieved
    phi_max = 20*pi/180;
    P.phi_max = phi_max;
    % pick natural frequency to achieve delta_a_max for step of phi_max
    zeta_roll = 2;
    
    % set control gains based on zeta and wn
    P.roll_kp = delta_a_max/phi_max;
    wn_roll = sqrt(P.roll_kp*a_phi2);
    P.roll_kd = (2*zeta_roll*wn_roll - a_phi1)/a_phi2;
    %P.roll_kd = P.roll_kd+.2; % add extra roll damping
    P.roll_ki = 0;
    
% select gains for course loop
   zeta_course = 0.9;
   wn_course = wn_roll/8;
   P.course_kp = 2*zeta_course*wn_course*P.Va0/P.gravity;
   P.course_ki = wn_course^2*P.Va0/P.gravity;
   P.course_kd = 0;
   
% select gains for sideslip hold
    % get transfer function data for delta_r to vr
    [num,den]=tfdata(T_v_delta_r,'v');
    a_beta2 = num(2);
    a_beta1 = den(2);
    % maximum possible rudder command
    delta_r_max = 20*pi/180;
    % Roll command when delta_r_max is achieved
    vr_max = 3;
    % pick natural frequency to achieve delta_a_max for step of phi_max
    zeta_beta = 0.707;
    P.beta_kp = delta_r_max/vr_max;
    wn_beta = (a_beta2*P.beta_kp+a_beta1)/2/zeta_beta;
    P.beta_ki = 0;%wn_beta^2/a_beta2;
    P.beta_kd = 0;

   
% select gains for the pitch loop
   % get transfer function delta_e to theta
   [num,den]=tfdata(T_theta_delta_e,'v');
   a_theta1 = den(2);
   a_theta2 = den(3);
   a_theta3 = num(3);
   % maximum possible elevator command
   delta_e_max = 45*pi/180;
   % Pitch command when delta_e_max is achieved
   theta_max = 10*pi/180;
   % pick natural frequency to achieve delta_e_max for step of theta_max
   zeta_pitch = 0.9;
   % set control gains based on zeta and wn
   P.pitch_kp = -delta_e_max/theta_max;
   wn_pitch = sqrt(a_theta2+P.pitch_kp*a_theta3);
   P.pitch_kd = (2*zeta_pitch*wn_pitch - a_theta1)/a_theta3;
   P.pitch_ki = 0.0;
   P.K_theta_DC = P.pitch_kp*a_theta3/(a_theta2+P.pitch_kp*a_theta3);

% select gains for altitude loop
   zeta_altitude = .9;%.707;
   wn_altitude = wn_pitch/40;
   P.altitude_kp = 2*zeta_altitude*wn_altitude/P.K_theta_DC/P.Va0;
   P.altitude_ki = wn_altitude^2/P.K_theta_DC/P.Va0;
%   P.altitude_kp = 0.0114;
%   P.altitude_ki = 0.0039;
   P.altitude_kd = 0;%-.001;
 
% airspeed hold using pitch
   [num,den]=tfdata(T_Va_theta,'v');
   a_V1 = den(2);
   zeta_airspeed_pitch = 1;%0.707;
   wn_airspeed_pitch = wn_pitch/10;
   P.airspeed_pitch_kp = (a_V1-2*zeta_airspeed_pitch*wn_airspeed_pitch)/P.K_theta_DC/P.gravity;
   P.airspeed_pitch_ki = -wn_airspeed_pitch^2/P.K_theta_DC/P.gravity;
 
% airspeed hold using throttle
   [num,den]=tfdata(T_Va_delta_t,'v');
   a_Vt1 = den(2);
   a_Vt2 = num(2);
   zeta_airspeed_throttle = 2;%0.707;
%    wn_airspeed_throttle = 5;   % a value of 5 causes instability...
   wn_airspeed_throttle = 3;
   P.airspeed_throttle_kp = (2*zeta_airspeed_throttle*wn_airspeed_throttle-a_Vt1)/a_Vt2;
   P.airspeed_throttle_ki = wn_airspeed_throttle^2/a_Vt2;
%   P.airspeed_throttle_integrator_gain = a_Vt1/a_Vt2/P.airspeed_throttle_ki;
 
% gains for slideslip
   P.sideslip_kp = .1;
   P.sideslip_kd = -.5;
   P.sideslip_ki = 0;
 
% TECS gains
    % throttle (unitless)
    P.TECS_E_kp = 1;
    P.TECS_E_ki = .5;
  
    % pitch command (unitless)
    P.TECS_L_kp = 1;
    P.TECS_L_ki = .1;
    
    % saturated altitude error
    P.TECS_h_error_max = 10; % meters

    
%---------------------------------------
% sensor parameters
    P.sigma_gyro = 0.13*pi/180; % standard deviation of gyros in rad/sec
    P.bias_gyro_x = 0;%0.1*pi/180*rand; % bias on x_gyro
    P.bias_gyro_y = 0;%0.1*pi/180*rand; % bias on y_gyro
    P.bias_gyro_z = 0;%0.1*pi/180*rand; % bias on z_gyro
    P.sigma_accel = 0.0025*9.8; % standard deviation of accelerometers in m/s^2
    P.sigma_static_pres = 0.01*1000; % standard deviation of static pressure sensor in Pascals
    P.sigma_diff_pres = 0.002*1000;  % standard deviation of diff pressure sensor in Pascals

% GPS parameters
    P.Ts_gps = 1; % sample rate of GPS in s
    P.beta_gps = 1/1100; % 1/s
    P.sigma_n_gps = 0.21;
    P.sigma_e_gps = 0.21; 
    P.sigma_h_gps = 0.40;
    P.sigma_Vg_gps = 0.05;
    P.sigma_course_gps = P.sigma_Vg_gps/P.Va0;



