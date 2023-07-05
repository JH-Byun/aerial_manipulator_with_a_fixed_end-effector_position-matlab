function out = getQdot(Theta,Thetadot)
phi = Theta(1,1);
theta = Theta(2,1);
psi = Theta(3,1);
phidot = Thetadot(1,1);
thetadot = Thetadot(2,1);
psidot = Thetadot(3,1);

out = [0 0 -cos(theta)*thetadot;...
    0 -sin(phi)*phidot cos(phi)*cos(theta)*phidot-sin(phi)*sin(theta)*thetadot;...
    0 -cos(phi)*phidot -sin(phi)*cos(theta)*phidot-cos(phi)*sin(theta)*thetadot];
end