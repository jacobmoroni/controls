function [sys,x0,str,ts,simStateCompliance]...
				 = ball_dynamics(t,x,u,flag,P)
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

sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [P.z0; P.zdot0; P.theta0; P.thetadot0];

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
  F        = u(1);
  
  % system parameters randomly generated to make the 
  % system uncertain
  persistent m1
  persistent m2
  persistent l
  persistent g
  
  if t==0,
    alpha = 0.0;  % uncertainty parameter
    m1 = 0.35 * (1+2*alpha*rand-alpha);  % kg
    m2 = 2 * (1+2*alpha*rand-alpha);     % kg
    l = 0.5 * (1+2*alpha*rand-alpha); % m
    g = 9.81; % m/s^2
  end

 % F = m1*g*z/l+m2*g/2;
    zddot= (1/m1)*(m1*z*thetadot^2 - m1*g*sin(theta));

    thetaddot=(1/((m2*l^2/3) + (m1*z^2)))*(F*l*cos(theta) - 2*m1*z*zdot*thetadot...
    - m1*g*z*cos(theta) - .5*m2*g*l*cos(theta));

sys = [zdot; zddot; thetadot; thetaddot];

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
sys = [z; theta];

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
