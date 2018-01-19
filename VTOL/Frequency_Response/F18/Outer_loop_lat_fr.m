%Frequency Response for lateral outer loop VTOL
bodeoptions('cstprefs');

s = tf('s');     % Define the Laplace variable s

G1 = -14.715/s
G2 = 1/(1.5*s+.1)
G = G1*G2
figure(2); clf;
bode(G,{0.01,1000});
grid on;
