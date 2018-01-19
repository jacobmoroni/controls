function [sys,x0,str,ts,simStateCompliance]...
				 = Whirly_dynamics(t,x,u,flag,P)
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
x0  = [P.phi0; P.phidot0; P.psi0; P.psi0; P.theta0; P.thetadot0];

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
  phi      = x(1);
  phidot   = x(2);
  psi      = x(3);
  psidot   = x(4);
  theta    = x(5);
  thetadot = x(6);

  PWM_left  = u(1);
  PWM_right = u(2);
  

  
  % system parameters randomly generated to make the 
  % system uncertain
  persistent g
  persistent l1
  persistent l2
  persistent m1
  persistent m2
  persistent d
  persistent h
  persistent r
  persistent Jx
  persistent Jy
  persistent Jz
  persistent km
  persistent siggyro
  persistent sigpix

  if t==0,
    alpha = 0.1;  % uncertainty parameter
      
    g = 9.81; %m/s2
    l1 = .85* (1+2*alpha*rand-alpha);  % m
    l2 = .3048* (1+2*alpha*rand-alpha); % m
    m1 = 0.891* (1+2*alpha*rand-alpha); %kg
    m2 = 1* (1+2*alpha*rand-alpha); %kg
    d = 0.178* (1+2*alpha*rand-alpha); %m
    h = 0.65* (1+2*alpha*rand-alpha); %m
    r = 0.12* (1+2*alpha*rand-alpha); %m
    Jx = 0.0047* (1+2*alpha*rand-alpha); %kg m^2
    Jy = 0.0014* (1+2*alpha*rand-alpha); %kg m^2
    Jz = 0.0041* (1+2*alpha*rand-alpha); %kg m^2
    km = 5.8;%* (1+2*alpha*rand-alpha); %N/PWM
    siggyro = 8.7266*10^-5* (1+2*alpha*rand-alpha); %rad
    sigpix = 0.05* (1+2*alpha*rand-alpha); %pix
    
 
  end

  % EOM for Whirly Bird

fl = PWM_left*km;
fr = PWM_right*km;
  
% fl=.5*F+tau/(2*d);
% fr=.5*F-tau/(2*d);
  
c= [(-thetadot^2*(Jz-Jy)*sin(phi)*cos(phi) + psidot^2*(Jz-Jy)*sin(phi)*cos(phi)*cos(theta)^2 ...
    -thetadot*psidot*cos(theta)*(Jx-(Jz-Jy)*(cos(phi)^2-sin(phi)^2)));...
    (psidot^2*sin(theta)*cos(theta)*(-Jx+m1*l1^2+m2*l2^2+Jy*sin(phi)^2 + Jz*cos(phi)^2)...
    -2*phidot*thetadot*(Jz-Jy)*sin(phi)*cos(phi)-phidot*psidot*cos(theta)*(-Jx+(Jz-Jy)*(cos(phi)^2-sin(phi)^2)));...
    (thetadot^2*(Jz-Jy)*sin(phi)*cos(phi)*sin(theta)-phidot*thetadot*cos(theta)*(Jx+(Jz-Jy)*(cos(phi)^2-sin(phi)^2))...
    -2*phidot*psidot*(Jz-Jy)*cos(theta)^2*sin(phi)*cos(phi)+2*thetadot*psidot*sin(theta)*cos(theta)*(Jx-m1*l1^2-m2*l2^2-Jy*sin(phi)^2-Jz*cos(phi)^2))];

M = [ Jx, 0, -Jx*sin(theta); ...
	   0,  (m1*l1^2 + m2*l2^2 + Jy*(cos(phi)^2) + Jz*(sin(phi)^2)), ((Jy-Jz)*sin(phi)*cos(phi)*cos(theta)); ...
	   -Jx*sin(theta), ((Jy-Jz)*sin(phi)*cos(phi)*cos(theta)), ((m1*l1^2 + m2*l2^2 +Jy*(sin(phi))^2 + Jz*(cos(phi))^2)*(cos(theta))^2 + Jx*(sin(theta))^2) ];

dPdq = [0; ((m1*l1 - m2*l2)*g*cos(theta)); 0];

Q = [d*(fl-fr); l1*(fl+fr)*cos(phi); (l1*(fl+fr)*cos(theta)*sin(phi)+d*(fr-fl)*sin(theta))];

qddot = (M)\(Q - c - dPdq);

phiddot = qddot(1);
thetaddot = qddot(2);
psiddot = qddot(3);


sys = [phidot; phiddot; psidot; psiddot; thetadot; thetaddot];

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
    psi        = x(3);
    theta      = x(5);
sys = [phi; psi; theta];

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
