% camera
%
% simulates camera
% input is 
%    uu(1:3) - target position
%    uu(4:6) - target velocity
%    uu(7:18) - MAV states
%
% output is 
%    px = x-pixel
%    py = y-pixel
%    ps = size of blob in image plane (in pixels)
%
% modified 11/30/2006 - Randy Beard
% modified 05/04/2007 - Randy Beard
% modified 05/04/2010 - Randy Beard
%
function out = point_gimbal(uu,P)
  
    % process inputs
    NN = 0;
    tn    = uu(1+NN); % target North position
    te    = uu(2+NN); % target East position
    td    = uu(3+NN); % target Down position
    NN = NN + 6;
    pn    = uu(1+NN); % MAV North position
    pe    = uu(2+NN); % MAV East position
    pd    = uu(3+NN); % MAV Down position
    phi   = uu(7+NN); % MAV roll angle
    theta = uu(8+NN); % MAV pitch angle
    psi   = uu(9+NN); % MAV yaw angle
    NN = NN + 12;
    az    = uu(1+NN); % gimbal azimuth angle
    el    = uu(2+NN); % gimbal elevation angle
    
    % line of sight vector in the inertial frame
    ell_v =  [tn-pn; te-pe; td-pd];
    
    % rotate line-of-sight vector into the body frame
    ell_b =  1/norm(ell_v) * Rot_v_to_b(phi, theta, psi) * ell_v;
    
    az_d =  atan2(ell_b(2), ell_b(1));
    el_d =  -asin(ell_b(3));
    
    az_d * 180/pi
    % proportional control for gimbal
    u_az = P.k_az * (az_d - az);
    u_el = P.k_el * (el_d - el);
    
    
    % create output
    out = [u_az; u_el];
end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_v_to_b(phi,theta,psi)
    % Rotation matrix from body coordinates to vehicle coordinates

    Rot_v_to_v1 = [...
        cos(psi), sin(psi), 0;...
        -sin(psi), cos(psi), 0;...
        0, 0, 1;...
        ];
    
    Rot_v1_to_v2 = [...
        cos(theta), 0, -sin(theta);...
        0, 1, 0;...
        sin(theta), 0, cos(theta);...
        ];
    
    Rot_v2_to_b = [...
        1, 0, 0;...
        0, cos(phi), sin(phi);...
        0, -sin(phi), cos(phi);...
        ];
    
    R = Rot_v2_to_b * Rot_v1_to_v2 * Rot_v_to_v1;

end

%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_b_to_g(az,el)
    % Rotation matrix from body coordinates to gimbal coordinates
    Rot_b_to_g1 = [...
        cos(az), sin(az), 0;...
        -sin(az), cos(az), 0;...
        0, 0, 1;...
        ];

    Rot_g1_to_g = [...
        cos(el), 0, -sin(el);...
        0, 1, 0;...
        sin(el), 0, cos(el);...
        ];

    R = Rot_g1_to_g * Rot_b_to_g1;

end
