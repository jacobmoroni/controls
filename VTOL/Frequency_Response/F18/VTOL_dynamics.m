function [sys,x0,str,ts,simStateCompliance]...
				 = VTOL_dynamics(t,x,u,flag,P)
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

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
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=================================================================
% mdlInitializeSizes
%=================================================================
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

sizes = simsizes;

sizes.NumContStates  = 6;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [P.z0; P.zdot0; P.theta0; P.thetadot0; P.h0; P.hdot0];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=================================================================
%
function sys=mdlDerivatives(t,x,u)
  z        = x(1);
  zdot     = x(2);
  theta    = x(3);
  thetadot = x(4);
  h        = x(5);
  hdot     = x(6);
  F        = u(1);
  T        = u(2);

  
  % system parameters randomly generated to make the 
  % system uncertain
  persistent mc
  persistent Jc
  persistent mr
  persistent ml
  persistent d
  persistent mu
  persistent g
  
  if t==0
    alpha = 0.2;  % uncertainty parameter
    mc = 1* (1+2*alpha*rand-alpha);  % kg
    Jc = 0.0042* (1+2*alpha*rand-alpha); % kg/m^2
    mr = 0.25* (1+2*alpha*rand-alpha); %kg
    ml = 0.25* (1+2*alpha*rand-alpha); %kg
    d = 0.3* (1+2*alpha*rand-alpha); %m
    mu = 0.1* (1+2*alpha*rand-alpha); %kg/s
    g = 9.81; %m/s2
 
  end
  
    fr = u(1)/2+1/(2*d)*u(2);
    fl = u(1)/2-1/(2*d)*u(2);

    %wind disturbance force
    F_wind = .1; %N
    
    zddot = (1/(mc+2*mr))*(-mu*zdot - (fr+fl)*sin(theta)+F_wind);
    
    thetaddot =  (1/(Jc+2*mr*d^2))*(d*(fr-fl));
    
    hddot = (1/(mc+2*mr))*(-(mc+2*mr)*g + (fr+fl)*cos(theta));

sys = [zdot; zddot; thetadot; thetaddot; hdot; hddot];

% end mdlDerivatives

%
%=================================================================
% mdlUpdate
%=================================================================
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=================================================================
% mdlOutputs
% Return the block outputs.
%=================================================================
function sys=mdlOutputs(t,x,u)
    z        = x(1);
    theta    = x(3);
    h        = x(5);
sys = [z; theta; h];

% end mdlOutputs

%
%=================================================================
% mdlGetTimeOfNextVarHit
%=================================================================
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1; % Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
