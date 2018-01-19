function PWM = whirlyPWM(u,P)
F = u(1);
tau   = u(2);

fl=.5*F+tau/(2*P.d);
fr=.5*F-tau/(2*P.d);

% km=6.293; %max force per prop
PWM_left = fl/P.km;
PWM_right = fr/P.km;

PWM  = [PWM_left PWM_right];
end