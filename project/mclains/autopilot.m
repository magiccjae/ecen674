function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   11/14/2014 - RWB
%   

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
%    phi_c_ff = uu(4+NN);  % feedforward roll command (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    
    % hack since no phi_c_ff is included in inputs in Simulink model
    phi_c_ff = 0;
    
    autopilot_version = 4;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- standard autopilot defined in book
        % autopilot_version == 3 <- Total Energy Control for longitudinal AP
        % autopilot_version == 4 <- no state machine
    switch autopilot_version
        case 1,
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 2,
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,phi_c_ff,Va,h,chi,phi,theta,p,q,r,t,P);
        case 3,
               [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,phi_c_ff,Va,h,chi,phi,theta,p,q,r,t,P);
        case 4,
               [delta, x_command] = autopilot_no_state_machine(Va_c,h_c,chi_c,phi_c_ff,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_tuning
%   - used to tune each loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    mode = 5;
    switch mode
        case 1, % tune the roll loop
            phi_c = chi_c; % interpret chi_c to autopilot as course command
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 2, % tune the course loop
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
            else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
            end                
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 3, % tune the throttle to airspeed loop and pitch loop simultaneously
            theta_c = 20*pi/180 + h_c;
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 4, % tune the pitch to airspeed loop 
            chi_c = 0;
            delta_t = P.u_trim(4);
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 5, % tune the pitch to altitude loop 
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = altitude_hold(h_c, h, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = altitude_hold(h_c, h, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
      end
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_uavbook
%   - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,phi_c_ff,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        phi_c   = phi_c_ff+course_hold(chi_c, chi, r, 1, P);
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
    else
        phi_c   = phi_c_ff+course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    persistent initialize_integrator;
    % initialize persistent variable
    if t==0,
        if h<=P.altitude_take_off_zone,     
            altitude_state = 1;
        elseif h<=h_c-P.altitude_hold_zone, 
            altitude_state = 2;
        elseif h>=h_c+P.altitude_hold_zone, 
            altitude_state = 3;
        else
            altitude_state = 4;
        end
        initialize_integrator = 1;
    end
    
    % implement state machine
    switch altitude_state,
        case 1,  % in take-off zone
            delta_t = P.climb_out_trottle;
            theta_c = P.theta_c_max;
            if h>=P.altitude_take_off_zone,
                altitude_state = 2;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
            
        case 2,  % climb zone
            delta_t = P.climb_out_trottle;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            if h>=h_c-P.altitude_hold_zone,
                altitude_state = 4;
                initialize_integrator = 1;
            elseif h<=P.altitude_take_off_zone,
                altitude_state = 1;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
            
        case 3, % descend zone
            delta_t = 0;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            if h<=h_c+P.altitude_hold_zone,
                altitude_state = 4;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
        case 4, % altitude hold zone
            delta_t = airspeed_with_throttle_hold(Va_c, Va, initialize_integrator, P);
            theta_c = altitude_hold(h_c, h, initialize_integrator, P);
            if h<=h_c-P.altitude_hold_zone,
                altitude_state = 2;
                initialize_integrator = 1;
            elseif h>=h_c+P.altitude_hold_zone,
                altitude_state = 3;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
    end
    
    delta_e = pitch_hold(theta_c, theta, q, P);
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_TECS
%   - longitudinal autopilot based on total energy control systems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,phi_c_ff,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = phi_c_ff+course_hold(chi_c, chi, r, 1, P);

    else
        phi_c   = phi_c_ff+course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot based on total energy control
    
    persistent E_integrator;
    persistent L_integrator;
    persistent E_error_d1;
    persistent L_error_d1;
    persistent delta_t_d1;
    persistent theta_c_d1;
    % initialize persistent variables at beginning of simulation
    if t==0,
        E_integrator = 0; 
        L_integrator = 0; 
        E_error_d1 = 0;
        L_error_d1 = 0;
        delta_t_d1 = 0;
        theta_c_d1 = 0;
    end
  
    % error in kinetic energy
    K_error = 0.5*P.mass*(Va_c^2-Va^2);
    K_ref = 0.5*P.mass*Va_c^2;
    
    % (saturated) error in potential energy
    U_error = P.mass*P.gravity*sat(h_c-h,P.TECS_h_error_max,-P.TECS_h_error_max);
    %U_error = P.mass*P.gravity*(h_c-h);
    
    % (normalized) error in total energy and energy difference
    E_error = (K_error+U_error)/K_ref;
    L_error = (U_error-K_error)/K_ref;
    
    % update the integrator (with anti-windup)
    if delta_t_d1>0 & delta_t_d1<1,
      E_integrator = E_integrator + (P.Ts/2)*(E_error + E_error_d1); % trapazoidal rule
    end
    if theta_c_d1>-P.theta_c_max & theta_c_d1<P.theta_c_max,
        L_integrator = L_integrator + (P.Ts/2)*(L_error + L_error_d1); % trapazoidal rule
    end
  
 
    delta_t = sat( P.TECS_E_kp*E_error + P.TECS_E_ki*E_integrator, 1, 0);
    theta_c = sat( P.TECS_L_kp*L_error + P.TECS_L_ki*L_integrator, P.theta_c_max, -P.theta_c_max);


    E_error_d1   = E_error;
    L_error_d1   = L_error;
    delta_t_d1 = delta_t;
    theta_c_d1 = theta_c;
    
    delta_e = pitch_hold(theta_c, theta, q, P);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_no_state_machine.  Works well for overpowered aerosonde
%   - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_no_state_machine(Va_c,h_c,chi_c,phi_c_ff,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        phi_c   = sat(phi_c_ff+course_hold(chi_c, chi, r, 1, P),P.phi_max,-P.phi_max);
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
    else
        phi_c   = sat(phi_c_ff+course_hold(chi_c, chi, r, 0, P),P.phi_max,-P.phi_max);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    if t==0,
        delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
        theta_c = altitude_hold(h_c, h, 1, P);
    else
        delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
        theta_c = altitude_hold(h_c, h, 0, P);
    end
    delta_e = pitch_hold(theta_c, theta, q, P);
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% course_hold
%   - regulate heading using the roll command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c = course_hold(chi_c, chi, r, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; % error at last sample (d1-delayed by one sample)
  end
 
  % compute the current error
  error = chi_c - chi;
  
  % update the integrator
  if abs(error)>15*pi/180,
      integrator = 0;
  else
      integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  end
  
  % proportional term
  up = P.course_kp * error;
  
  % integral term
  ui = P.course_ki * integrator;
  
  % derivative term
  ud = -P.course_kd*r;
  
  
  % implement PID control
  phi_c = sat(up + ui + ud, 45*pi/180, -45*pi/180);
  
  % implement integrator anti-windup
  if P.course_ki~=0,
    phi_c_unsat = up+ui+ud;
    k_antiwindup = P.Ts/P.course_ki;
    integrator = integrator + k_antiwindup*(phi_c-phi_c_unsat);
  end

  % update persistent variables
  error_d1 = error;

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_hold
%   - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, P)
 
  % compute the current error
  error = phi_c - phi;
  
  % proportional term
  up = P.roll_kp * error;
  
  % derivative term
  ud = -P.roll_kd*p;
  
  % implement PID control
  delta_a = sat(up + ud, 45*pi/180, -45*pi/180);
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_hold
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_hold(theta_c, theta, q, P)
 
  % compute the current error
  error = theta_c - theta;
  
  % proportional term
  up = P.pitch_kp * error;
  
  % derivative term
  ud = -P.pitch_kd * q;
  
  % implement PID control
  delta_e = sat(up + ud, 45*pi/180, -45*pi/180);
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_pitch_hold
%   - regulate airspeed using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
  end
 
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = P.airspeed_pitch_kp * error;
  
  % integral term
  ui = P.airspeed_pitch_ki * integrator;
  
  % implement PID control
  theta_c = sat(up + ui, P.theta_c_max, -P.theta_c_max);
  
  % implement integrator antiwindup
  if P.airspeed_pitch_ki~=0,
    theta_c_unsat = up + ui;
    k_antiwindup = P.Ts/P.airspeed_pitch_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end

  % update persistent variables
  error_d1 = error;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle_hold
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
  end
 
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
    
  % proportional term
  up = P.airspeed_throttle_kp * error;
  
  % integral term
  ui = P.airspeed_throttle_ki * integrator;
    
  % implement PID control
  delta_t = sat(P.u_trim(4)+up + ui, 1, 0);
  
  % implement integrator anti-windup
  if P.airspeed_throttle_ki~=0,
    delta_t_unsat = P.u_trim(4) + up + ui;
    k_antiwindup = P.Ts/P.airspeed_throttle_ki;
    integrator = integrator + k_antiwindup*(delta_t-delta_t_unsat);
  end
  
  % update persistent variables
  error_d1 = error;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude_hold
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = altitude_hold(h_c, h, flag, P)
  persistent integrator;
  persistent error_d1;
  persistent hdot;
  persistent hdot_d1;
  persistent h_d1;

  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
      hdot = 0;
      hdot_d1 = 0;
      h_d1 = 0;
  end
 
  % compute the current error
  error = h_c - h;
  
  % update the integrator
    integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  hdot = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*hdot_d1...
      + (2/(2*P.tau+P.Ts))*(h - h_d1);

  % proportional term
  up = P.altitude_kp * error;
  
  % integral term
  ui = P.altitude_ki * integrator;
  
  % derivative gain
  ud = P.altitude_kd * hdot;
  
  % implement PID control
    theta_c = sat(up + ui + ud, P.theta_c_max, -P.theta_c_max);
  
  % implement integrator anti-windup
  if P.altitude_ki~=0,
    theta_c_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.altitude_ki;
    integrator = integrator + k_antiwindup*(theta_c-theta_c_unsat);
  end
  
  % update persistent variables
  error_d1 = error;
  hdot_d1 = hdot;
  h_d1 = h;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coordinated_turn_hold
%   - sideslip with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_r = coordinated_turn_hold(v, flag, P)
  persistent integrator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      error_d1   = 0; 
  end
 
  % compute the current error
  error = -v;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % proportional term
  up = P.sideslip_kp * error;
  
  % integral term
  ui = P.sideslip_ki * integrator;
  
  % derivative term
  ud = 0;%-P.sideslip_kd * r;
  
  
  % implement PID control
  theta_r = sat(up + ui + ud, 30*pi/180, -30*pi/180);
  
  % implement integrator antiwindup
  if P.sideslip_ki~=0,
    theta_r_unsat = up + ui + ud;
    k_antiwindup = P.Ts/P.sideslip_ki;
    integrator = integrator + k_antiwindup*(theta_r-theta_r_unsat);
  end
  
  % update persistent variables
  error_d1 = error;
 
end

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end
  
 