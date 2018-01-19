% syms s
% 
% eq= s^3+15*s^2+60*s+120
% simplify(eq)
% factor(eq)
% roots(eq)
% p=[1 15 60 120];
% q=[3 6 4 9];
% r=[1 4 3];
% roots(r)

syms s R L k c m J b
% R=1;
% L=1;
% k=1;
% c=1;
% m=1;
% J=1;
% b=1;
% eq=(L/R+1/s+k.^2/((m+J/R.^2)*s.^2*R)+k.^2/b)*(k*s*R.^2/c+k/((m+J/R.^2)*s)+k/b)
% eq2=expand(eq)
% eq3=simplify(eq2)
A=[s-R/L -k/L 0; k/(m+J/R^2) s-b/(m+J/R^2) -c/(R*(m+J/R^2)); 0 1/R s];
B=[s-R/L -k/L 1/L; k/(m+J/R^2) s-b/(m+J/R^2) 0; 0 1/L 0];
Transfer=det(B)/det(A)
simplify (Transfer)
