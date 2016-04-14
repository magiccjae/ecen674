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
function out = camera(uu,P)
  
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
    NN = NN + 2;
    t     = uu(1+NN); % time
    
    % persistent variable
    persistent blob_handle;  % figure handle for blob
    
    % determine pixel location and size
    [eps_x, eps_y, eps_s] = cameraProjection(tn,te,td,pn,pe,pd,phi,theta,psi,az,el,P);
  
    
    if t==0,  % initialize the plot
        figure(4), clf, hold on
    
        % plot camera view 
        h=subplot('position',[0.1, 0.1, 0.8, 0.8]);
        tmp = P.cam_pix/2;
        set(h,'XAxisLocation','top','XLim',[-tmp,tmp],'YLim',[-tmp,tmp]);
        axis ij
        hold on
        blob_handle = drawBlob([eps_y, eps_x], eps_s, [], 'normal');
        xlabel('px (pixels)')
        ylabel('py (pixels)')
        title('Camera View')
    elseif eps_y~=-9999 & eps_x~=-9999,
        drawBlob([eps_y, eps_x], eps_s, blob_handle);
    end
    
    % create output
    out = [eps_x; eps_y; eps_s];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle=drawBlob(z, R, handle, mode)

  th = 0:.1:2*pi;
  X = z(1)+ R*cos(th);
  Y = z(2)+ R*sin(th);
  
  if isempty(handle),
    handle = fill(Y, X, 'r', 'EraseMode', mode);
  else
    set(handle,'XData',Y,'YData',X);
  end
  
end


%=======================================================================
% cameraProjection
% project the target onto the camera
%==============================2=========================================
function [eps_x, eps_y, eps_size] = cameraProjection(tn,te,td,pn,pe,pd,phi,theta,psi,az,el,P)

    % target in the vehicle frame
    p_obj_v = [tn; te; td] - [pn; pe; pd];
  
    % tranform to the camera frame
    R_v_b = Rot_v_to_b(phi,theta,psi);  % vehicle to body
    R_b_g = Rot_b_to_g(az,el);          % body to gimbal
    R_g_c = [...                        % gimbal to camera
        0, 1, 0;...
        0, 0, 1;...
        1, 0, 0];
    p_obj_c = (R_g_c * R_b_g * R_v_b) * p_obj_v;
    
    % convert to pixel location
    if p_obj_c(3)<.1,
        eps_x=-9999;
        eps_y=-9999;
        eps_size = 0;
    else
        eps_x =  P.f*(p_obj_c(1)/(p_obj_c(3)))+P.pixelnoise*randn;
        eps_y =  P.f*(p_obj_c(2)/(p_obj_c(3)))+P.pixelnoise*randn;
        eps_size = P.f*(P.target_size/(p_obj_c(3)))+P.pixelnoise*randn;
    end
  
    % snap output of camera to -9999 if outside field of view
    tmp = P.cam_pix/2+eps_size;
    if eps_x<-tmp | eps_x>tmp | eps_y<-tmp | eps_y>tmp,
        eps_x = -9999;
        eps_y = -9999;
        eps_size = 0;
    end
end
  
  
%%%%%%%%%%%%%%%%%%%%%%%
function R = Rot_v_to_b(phi,theta,psi);
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
function R = Rot_b_to_g(az,el);
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

