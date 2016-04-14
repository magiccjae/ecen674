function drawEnvironment(uu,V,F,colors,map,R_min,cam_fov)

    % process inputs to function
    NN = 0;
    pn       = uu(1+NN);       % inertial North position     
    pe       = uu(2+NN);       % inertial East position
    pd       = uu(3+NN);       % inertial Down position
    u        = uu(4+NN);       % body frame velocities
    v        = uu(5+NN);       
    w        = uu(6+NN);       
    phi      = uu(7+NN);       % roll angle         
    theta    = uu(8+NN);       % pitch angle     
    psi      = uu(9+NN);       % yaw angle     
    p        = uu(10+NN);      % roll rate
    q        = uu(11+NN);      % pitch rate     
    r        = uu(12+NN);      % yaw rate   
    NN = NN + 12;
    t        = uu(1+NN);       % time
    NN = NN + 1;
    az       = uu(1+NN);       % gimbal azimuth angle
    el       = uu(2+NN);       % gimbal elevation angle
    NN = NN + 2;
    target   = [uu(1+NN); uu(2+NN); uu(3+NN)]; % target position
    NN = NN + 6;
    path     = uu(1+NN:13+NN); 
    NN = NN + 13;
    num_waypoints = uu(1+NN);
    waypoints     = reshape(uu(2+NN:5*num_waypoints+1+NN),5,num_waypoints)'; 
 
    % define persistent variables 
    persistent aircraft_handle;  % figure handle for MAV
    persistent path_handle;      % handle for straight-line or orbit path
    persistent waypoint_handle;  % handle for waypoints
    persistent target_handle;    % handle for target
    persistent fov_handle;       % handle for camera field-of-view

    S = 500; % plot size
    
    % first time function is called, initialize plot and persistent vars
    if t==0,

        figure(1), clf
                
        aircraft_handle = drawBody(V,F,colors,...
                                   pn,pe,pd,phi,theta,psi,...
                                   [], 'normal');
        hold on
        waypoint_handle = drawWaypoints(waypoints, R_min, [], 'normal');
        path_handle = drawPath(path, S/4, [], 'normal');
        drawMap(map);
        target_handle = drawTarget(target, map.StreetWidth/12, [], 'normal');
        fov_handle = drawFov(pn, pe, pd, phi, theta, psi,az,el,cam_fov,map,[],'normal');
        
        title('UAV')
        axis([0,map.width,0,map.width,0,-2*pd]);
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(-40,70)  % set the view angle for figure
        grid on
        
        
    % at every other time step, redraw MAV
    else 
        drawBody(V,F,colors,...
                     pn,pe,pd,phi,theta,psi,...
                     aircraft_handle);
        drawWaypoints(waypoints, R_min, waypoint_handle);
        drawPath(path, S, path_handle);
        drawTarget(target, map.StreetWidth/12, target_handle);
        drawFov(pn, pe, pd, phi, theta, psi,az,el,cam_fov,map,fov_handle);

    end
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawFov(pn, pe, pd, phi, theta, psi,az,el,cam_fov,map,handle, mode)
                           

    %-------vertices and faces for camera field-of-view --------------
    % vertices
    % define unit vectors along fov in the camera gimbal frame
    pts = [...
        cos(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2);...
        cos(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2);...
        cos(cam_fov/2)*cos(cam_fov/2), -sin(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2);...
        cos(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2)*cos(cam_fov/2),  sin(cam_fov/2);...
        ]';
    % transform from gimbal coordinates to the vehicle coordinates
    pts = Rot_v_to_b(phi,theta,psi)'*Rot_b_to_g(az,el)'*pts;

    % first vertex is at center of MAV vehicle frame
    Vert = [pn, pe, pd];  
    % project field of view lines onto ground plane and make correction
    % when the projection is above the horizon
    for i=1:4,
        % alpha is the angle that the field-of-view line makes with horizon
        alpha = atan2(pts(3,i),norm(pts(1:2,i)));
        if alpha > 0,
            % this is the normal case when the field-of-view line
            % intersects ground plane
            Vert = [...
                Vert;...
                [pn-pd*pts(1,i)/pts(3,i), pe-pd*pts(2,i)/pts(3,i), 0];...
                ];
        else
            % this is when the field-of-view line is above the horizon.  In
            % this case, extend to a finite, but far away (9999) location.
            Vert = [...
                Vert;...
                [pn+9999*pts(1,i), pe+9999*pts(2,i),0];...
            ];
        end
    end

    Faces = [...
          1, 1, 2, 2;... % x-y face
          1, 1, 3, 3;... % x-y face
          1, 1, 4, 4;... % x-y face
          1, 1, 5, 5;... % x-y face
          2, 3, 4, 5;... % x-y face
        ];

    edgecolor      = [1, 1, 1]; % black
    footprintcolor = [0,0,0];%[1,0,1];%[1,1,0];
    colors = [edgecolor; edgecolor; edgecolor; edgecolor; footprintcolor];  

  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  Vert = Vert*R;

  if isempty(handle),
    handle = patch('Vertices', Vert, 'Faces', Faces,...
                 'FaceVertexCData',colors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',Vert,'Faces',Faces);
  end
  
end 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawBody(V,F,colors,...
                               pn, pe, pd, phi, theta, psi,...
                               handle, mode)
  V = rotate(V', phi, theta, psi)';  % rotate rigid body  
  V = translate(V', pn, pe, pd)';  % translate after rotation

  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;

  if isempty(handle),
    handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',colors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
  
end 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawPath(path, S, handle, mode)
    flag = path(1); 
    r    = [path(3); path(4); path(5)];
    q    = [path(6); path(7); path(8)];
    c    = [path(9); path(10); path(11)];
    rho  = path(12);
    lam  = path(13);

    switch flag,
        case 1,
            XX = [r(1), r(1)+S*q(1)];
            YY = [r(2), r(2)+S*q(2)];
            ZZ = [r(3), r(3)+S*q(3)];
        case 2,
            N = 100;
            th = [0:2*pi/N:2*pi];
            XX = c(1) + rho*cos(th);
            YY = c(2) + rho*sin(th);
            ZZ = c(3)*ones(size(th));
    end
    
    if isempty(handle),
        handle = plot3(YY,XX,-ZZ,'r', 'EraseMode', mode);
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow
    end
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawWaypoints(waypoints, R_min, handle, mode)

    if waypoints(1,4)==-9999, % check to see if Dubins paths
        XX = [waypoints(:,1)];
        YY = [waypoints(:,2)];
        ZZ = [waypoints(:,3)];
    else
        XX = [];
        YY = [];
        for i=2:size(waypoints,1),
            dubinspath = dubinsParameters(waypoints(i-1,:),waypoints(i,:),R_min);
            [tmpX,tmpY] = pointsAlongDubinsPath(dubinspath,0.1);
            XX = [XX; tmpX];
            YY = [YY; tmpY];     
        end
        ZZ = waypoints(i,3)*ones(size(XX));
    end
    
    if isempty(handle),
        handle = plot3(YY,XX,-ZZ,'b', 'EraseMode', mode);
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow
    end
end 


%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];

  % rotate vertices
  XYZ = R_yaw*R_pitch*R_roll*XYZ;
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)

  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
  
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% drawMap
%   plot obstacles and path
function drawMap(map)%,path,smoothedPath,tree,R_min)
  
 
  % draw buildings 
  V = [];
  F = [];
  patchcolors = [];
  count = 0;
  for i=1:map.NumBlocks,
      for j=1:map.NumBlocks,
        [Vtemp,Ftemp,patchcolorstemp] = buildingVertFace(map.buildings_n(i),...
            map.buildings_e(j),map.BuildingWidth,map.heights(j,i));
        V = [V; Vtemp];
        Ftemp = Ftemp + count;
        F = [F; Ftemp];
        count = count + 8;
        patchcolors = [patchcolors;patchcolorstemp];
      end
  end
  
  patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat');
 

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% buildingVertFace(x,y,width,height)
%%   define patches for a building located at (x,y)
function [V,F,patchcolors] = buildingVertFace(n,e,width,height)
 
  % vertices of the building
  V = [...
        e+width/2, n+width/2, 0;...
        e+width/2, n-width/2, 0;...
        e-width/2, n-width/2, 0;...
        e-width/2, n+width/2, 0;...
        e+width/2, n+width/2, height;...
        e+width/2, n-width/2, height;...
        e-width/2, n-width/2, height;...
        e-width/2, n+width/2, height;...
        ];    
  % define faces of fuselage
  F = [...
        1, 4, 8, 5;... % North Side
        1, 2, 6, 5;... % East Side
        2, 3, 7, 6;... % South Side
        3, 4, 8, 7;... % West Side
        5, 6, 7, 8;... % Top
        ];   

  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1,1,0];
  mymagenta   = [0, 1, 1];

  patchcolors = [...
    mygreen;... % North
    mygreen;... % East
    mygreen;... % South
    mygreen;... % West
    myyellow;...  % Top
    ];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pointsAlongDubinsPath
%%   Find points along Dubin's path separted by Del (to be used in
%%   collision detection)
function [X,Y] = pointsAlongDubinsPath(dubinspath,Del)


  % points along start circle
  th1 = mod(atan2(dubinspath.ps(2)-dubinspath.cs(2),dubinspath.ps(1)-dubinspath.cs(1)),2*pi);
  th2 = mod(atan2(dubinspath.w1(2)-dubinspath.cs(2),dubinspath.w1(1)-dubinspath.cs(1)),2*pi);
  if dubinspath.lams>0,
      if th1>=th2,
        th = [th1:Del:2*pi,0:Del:th2];
      else
        th = [th1:Del:th2];
      end
  else
      if th1<=th2,
        th = [th1:-Del:0,2*pi:-Del:th2];
      else
        th = [th1:-Del:th2];
      end
  end
  X = [];
  Y = [];
  for i=1:length(th),
    X = [X; dubinspath.cs(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.cs(2)+dubinspath.R*sin(th(i))];
  end
  
  % points along straight line 
  sig = 0;
  while sig<=1,
      X = [X; (1-sig)*dubinspath.w1(1) + sig*dubinspath.w2(1)];
      Y = [Y; (1-sig)*dubinspath.w1(2) + sig*dubinspath.w2(2)];
      sig = sig + Del;
  end
    
  % points along end circle
  th2 = mod(atan2(dubinspath.pe(2)-dubinspath.ce(2),dubinspath.pe(1)-dubinspath.ce(1)),2*pi);
  th1 = mod(atan2(dubinspath.w2(2)-dubinspath.ce(2),dubinspath.w2(1)-dubinspath.ce(1)),2*pi);
  if dubinspath.lame>0,
      if th1>=th2,
        th = [th1:Del:2*pi,0:Del:th2];
      else
        th = [th1:Del:th2];
      end
  else
      if th1<=th2,
        th = [th1:-Del:0,2*pi:-Del:th2];
      else
        th = [th1:-Del:th2];
      end
  end
  for i=1:length(th),
    X = [X; dubinspath.ce(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.ce(2)+dubinspath.R*sin(th(i))];
  end
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle=drawTarget(z, R, handle, mode);

  th = 0:.1:2*pi;
  X = z(1)+ R*cos(th);
  Y = z(2)+ R*sin(th);
  Z = z(3)*ones(length(th));
  
  if isempty(handle),
    handle = fill(Y, X, 'r', 'EraseMode', mode);
  else
    set(handle,'XData',Y,'YData',X);
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

  