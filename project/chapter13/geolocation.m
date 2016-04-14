% geolocation
%
% compute location of target given position in camera
% input is 
%    uu(1:3)   - camera data (eps_x, eps_y, eps_s)
%    uu(4:15)  - MAV states
%    uu(16:17) - gimbal azimuth, elevation 
%    uu(18)    - time
%
% output is 
%    tn - estimated North postion of target
%    te - estimated East position of target
%    L  - range to target
%
% modified 05/06/2010 - Randy Beard
%
function out = geolocation(in,P)
  
    % process inputs
    NN = 0;
    eps_x     = in(1+NN); % x-pixel
    eps_y     = in(2+NN); % y-pixel
    eps_s     = in(3+NN); % pixel size
    NN = NN + 3;
    pn        = in(1+NN);
    pe        = in(2+NN);
    pd        = -in(3+NN);
    % Va      = in(4+NN);
    % alpha   = in(5+NN);
    % beta    = in(6+NN);
    phi       = in(7+NN);
    theta     = in(8+NN);
    chi       = in(9+NN);
    % p       = in(10+NN);
    % q       = in(11+NN);
    % r       = in(12+NN);
    Vg        = in(13+NN);
    % wn      = in(14+NN);
    % we      = in(15+NN);
    psi     = in(16+NN);
    NN = NN + 16;
    az        = in(1+NN); % gimbal azimuth angle
    el        = in(2+NN); % gimbal elevation angle
    NN = NN + 2;
    t         = in(1+NN); % time

    
    %--------------------------------------
    % begin geolocation code 
    selector = 0;
    
    persistent tn_hat;
    persistent te_hat;
    persistent L_hat;
    if t==0
        tn_hat = 0;
        te_hat = 0;
        L_hat = 1;
    end
    if selector==1
        F = sqrt(P.f^2 + eps_x^2 + eps_y^2);
        ell_c_unit = 1/F * [eps_x; eps_y; P.f];
        Rot_c_to_g = [0 0 1; 1 0 0; 0 1 0];
        ell_i_unit = Rot_v_to_b(phi,theta,psi)' *  Rot_b_to_g(az,el)' * Rot_c_to_g * ell_c_unit;
        k_i_unit = [0; 0; 1];
        denominator = dot(k_i_unit, ell_i_unit);
        h = -pd;
        L = h / denominator;
        P_obj_i = [pn; pe; pd] + L * ell_i_unit;
        tn = P_obj_i(1);
        te = P_obj_i(2);
        
    else
        T_out = 0.01;
        N = 10;
        Q_geolocation = 100 * diag([1 1 1]);
        R_north = P.sigma_measurement_n^2;
        R_east = P.sigma_measurement_e^2;
        R_down = P.sigma_measurement_h^2;
        P_geolocation = zeros(3);
        % prediction step
        for i=1:N
            p_mav_dot = [Vg*cos(chi); Vg*sin(chi); 0];
            temp = [tn_hat-pn; te_hat-pe; 0-pd];
            f_x = [0;...
                   0;...
                   -temp'*p_mav_dot / L_hat];
            tn_hat = tn_hat + (T_out/N)*f_x(1);
            te_hat = te_hat + (T_out/N)*f_x(2);
            L_hat = L_hat + (T_out/N)*f_x(3);
            A = [0 0 0;...
                 0 0 0;...
                 -p_mav_dot(1)/L_hat -p_mav_dot(2)/L_hat temp'*p_mav_dot/L_hat^2];
            P_geolocation = P_geolocation + (T_out/N)*(A*P_geolocation + P_geolocation*A' + Q_geolocation);
        end
        % measurement update
        F = sqrt(P.f^2 + eps_x^2 + eps_y^2);
        ell_c_unit = 1/F * [eps_x; eps_y; P.f];
        Rot_c_to_g = [0 0 1; 1 0 0; 0 1 0];
        ell_i_unit = Rot_v_to_b(phi,theta,psi)' *  Rot_b_to_g(az,el)' * Rot_c_to_g * ell_c_unit;
        
        C_1 = [1 0 -ell_i_unit(1)];
        L_1 = P_geolocation * C_1' / (R_north + C_1*P_geolocation*C_1');
        P_geolocation = (eye(3) - L_1*C_1)*P_geolocation;
        h_1 = tn_hat - L_hat*ell_i_unit(1);
        tn_hat = tn_hat + L_1(1) * (pn - h_1);
        te_hat = te_hat + L_1(2) * (pn - h_1);
        L_hat = L_hat + L_1(3) * (pn - h_1);
        
        C_2 = [0 1 -ell_i_unit(2)];
        L_2 = P_geolocation * C_2' / (R_east + C_2*P_geolocation*C_2');
        P_geolocation = (eye(3) - L_2*C_2)*P_geolocation;
        h_2 = te_hat - L_hat*ell_i_unit(2);
        tn_hat = tn_hat + L_2(1) * (pe - h_2);
        te_hat = te_hat + L_2(2) * (pe - h_2);
        L_hat = L_hat + L_2(3) * (pe - h_2);
        
        C_3 = [0 0 -ell_i_unit(3)];
        L_3 = P_geolocation * C_3' / (R_down + C_3*P_geolocation*C_3');
        P_geolocation = (eye(3) - L_3*C_3)*P_geolocation;
        h_3 = -L_hat*ell_i_unit(3);
        tn_hat = tn_hat + L_3(1) * (pd - h_3);
        te_hat = te_hat + L_3(2) * (pd - h_3);
        L_hat = L_hat + L_3(3) * (pd - h_3);
        
        tn = tn_hat;
        te = te_hat;
        L = L_hat;
    end
    
    % end geolocation code 
    %--------------------------------------
    
    % create output
    out = [tn; te; L];
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
