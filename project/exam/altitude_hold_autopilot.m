function y = altitude_hold_autopilot(uu,P)

    h = uu(1);
    theta = uu(2);
    q = uu(3);
    h_c = uu(4);
    t = uu(5);
    
    if t==0
        h = P.h0;
        theta = P.theta0;
        q = P.q0;
    end
    
    [delta, x_command] = autopilot_uavbook(h_c,h,theta,q,t,P);
    
    y = [delta; x_command];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(h_c,h,theta,q,t,P)
  
    %----------------------------------------------------------
    % longitudinal autopilot
    theta_c = altitude_hold(h_c, h, P);    
    delta_e = pitch_hold(theta_c, theta, q, P);
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = delta_e;
    % commanded (desired) states
    x_command = theta_c;
            
    y = [delta; x_command];
 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_hold(theta_c, theta, q, P)
    persistent error_d1;
    if isempty(error_d1)       % reset (initialize) persistent variables when flag==1
        error_d1 = 0;   % _d1 means delayed by one time step
    end
    error = theta_c - theta;    % compute the current error\
    error_d1 = error; % update the error for next time through the loop
    delta_e = sat(...                   % implement PID control
        P.kp_theta * error - ...         % proportional term
        P.kd_theta * q,...  % derivative term
        P.delta_e_max, -P.delta_e_max ... % ensure abs(u)<=limit
        );
end

function theta_c = altitude_hold(h_c, h, P)
    persistent integrator;
    persistent error_d1;
    if isempty(integrator)        % reset (initialize) persistent variables when flag==1
        integrator = 0;
        error_d1 = 0;   % _d1 means delayed by one time step
    end
    error = h_c - h;    % compute the current error
    integrator = integrator + (P.Ts / 2) * (error + error_d1); % update integrator
    error_d1 = error; % update the error for next time through the loop
    theta_c = sat(...                   % implement PID control
        P.kp_h * error + ...         % proportional term
        P.ki_h * integrator, ...    % integral term
        P.theta_max, -P.theta_max ...    % ensure abs(u)<=limit
        );
    if P.ki_h~=0        % implement integrator anti-windup
        u_unsat = P.kp_h * error + P.ki_h * integrator;
        integrator = integrator + P.Ts / P.ki_h * (theta_c - u_unsat);
    end    
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