function out = Rz(psi)                  % rotz(psi)
    out = [cos(psi) -sin(psi) 0;
           sin(psi) cos(psi) 0;
           0 0 1];
end