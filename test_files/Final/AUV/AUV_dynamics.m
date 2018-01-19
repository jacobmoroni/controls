function [sys,x0,str,ts,simStateCompliance]...
				 = AUV_dynamics(t,x,u,flag,P)
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

sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [P.phi0; P.phidot0];

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
  phi        = x(1);
  phidot     = x(2);
  Tau        = u(1);
  
  % system parameters randomly generated to make the 
  % system uncertain
  persistent J
  persistent b
  persistent m 
  persistent L
  persistent g
  
  if t==0
    alpha = 0.0;  % uncertainty parameter
    J = 45* (1+2*alpha*rand-alpha); %kg*m^2
    b = 5* (1+2*alpha*rand-alpha); %N-m-s
    m = 200* (1+2*alpha*rand-alpha); %kg
    L = .01* (1+2*alpha*rand-alpha); %m
    g = 9.81; %m/s^2
  end
  
  phiddot= -b/J*phidot - m*g*L/J*sin(phi) + Tau;

sys = [phidot; phiddot];

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
    phi        = x(1);
sys = [phi];

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
