function out = getQ(Theta)
phi = Theta(1,1);
theta = Theta(2,1);
psi = Theta(3,1);
out = [1 0 -sin(theta);...
    0 cos(phi) sin(phi)*cos(theta);...
    0 -sin(phi) cos(phi)*cos(theta)];
end