function [sys,x0,str,ts] = target_motion(t,x,u,flag,P,map)
% define motion of target vehicle
% Modified 5/23/2007 - R. Beard
% Modified 2/26/2008 - R. Beard
%

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(P,map);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u,P,map);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(P,map)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 3; % position
sizes.NumDiscStates  = 6; % velocity, last position
sizes.NumOutputs     = 6; % output position and velocity
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
  % randomly pick an intersection
  block_n = ceil((map.NumBlocks-2)*rand)+1;
  block_e = ceil((map.NumBlocks-2)*rand)+1;
  % initial position of target
  z0 = [...
      block_n*(map.StreetWidth+map.BuildingWidth);
      block_e*(map.StreetWidth+map.BuildingWidth);
      0;...
      ];
  % randomly pick directions of motion
  temp = rand;
  if temp < 0.25,
      v0 = [P.target_velocity; 0; 0];
  elseif temp < 0.5,
      v0 = [0; P.target_velocity; 0];
  elseif temp < 0.75,
      v0 = [-P.target_velocity; 0; 0];
  else
      v0 = [0; -P.target_velocity; 0];
  end
        
   % continuous states
   NN = 0;
   x0(1+NN) = z0(1);
   x0(2+NN) = z0(2);
   x0(3+NN) = z0(3);
   % discrete states
   NN = 3;
   x0(1+NN) = v0(1);
   x0(2+NN) = v0(2);
   x0(3+NN) = v0(3);
   x0(4+NN) = z0(1);
   x0(5+NN) = z0(2);
   x0(6+NN) = z0(3);
x0 = x0';

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)
 
 vel = [x(4); x(5); x(6)];

 sys = vel;

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u,P,map)

   % continuous states
   z      = [x(1); x(2); x(3)];
   % discrete states
   vel    = [x(4); x(5); x(6)];
   z_last = [x(7); x(8); x(9)];

 
   if (norm(z-z_last)>=map.BuildingWidth+map.StreetWidth),        
  
      % randomly pick directions of motion
      temp = rand;
      if (temp < 0.25) & (z(1)<map.width-map.StreetWidth),
          vel = [P.target_velocity; 0; 0];
          z_last = z;
    elseif (temp < 0.5) & (z(2)<map.width-map.StreetWidth),
          vel = [0; P.target_velocity; 0];
          z_last = z;
      elseif (temp < 0.75) & (z(1) > map.StreetWidth),
          vel = [-P.target_velocity; 0; 0];
          z_last = z;
      elseif (z(2)>map.StreetWidth),
          vel = [0; -P.target_velocity; 0];
          z_last = z;
      else
          vel = [0; 0; 0];
      end

   end

    sys = [vel; z_last];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x)

   % continuous states
   z      = [x(1); x(2); x(3)];
   % discrete states
   vel      = [x(4); x(5); x(6)];

   sys = [z; vel];

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end 

%=============================================================================
% determine if z is at a road intersection
%=============================================================================
function flag = atIntersection(z, street_width, building_width)
  tmp = (z-street_width/2)/(street_width+building_width);
  if norm(tmp-round(tmp))<.1,
      flag = 1;
  else
      flag = 0;
  end
  
