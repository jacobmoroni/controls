% EOM for Whirly Bird

M = [ Jx, 0, -Jx*sin(theta); ...
	   0,  (m1*l1^2 + m2*l2^2 + Jy*(cos(phi)^2) + Jz*(sin(phi)^2)), ((Jy-Jz)*sin(phi)*cos(phi)*cos(theta)); ...
	   -Jx*sin(theta), ((Jy-Jz)*sin(phi)*cos(phi)*cos(theta)), ((m1*l1^2 + m2*l2^2 +Jy*(sin(phi))^2 + Jz*(cos(phi))^2)*(cos(theta))^2 + Jx*(sin(theta))^2) ];

dPdq = [0; ((m1*l1 - m2*l2)*g*cos(theta)); 0];

Q = [d*(fl-fr); l1*(fl+fr)*cos(phi); (l1*(fl+fr)*cos(theta)*sin(phi)+d*(fr-ft)*sin(theta))];

qddot = inv(M)*(Q - c - dPdq);

phiddot = qddot(1);
thetaddot = qddot(2);
psiddot = qddot(3);
