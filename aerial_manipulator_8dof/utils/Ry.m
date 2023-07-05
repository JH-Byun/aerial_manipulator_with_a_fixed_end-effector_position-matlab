function out = Ry(theta)                % roty(theta)
    out = [cos(theta) 0 sin(theta);
           0 1 0;
           -sin(theta) 0 cos(theta)];
end