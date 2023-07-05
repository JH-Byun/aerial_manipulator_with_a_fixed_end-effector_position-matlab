function [out1, out2] = mixer(in)
u_avant = in;
r = 0.275; % DJI F550
k_f = 0.240*9.8/460.76692^2;
k_m = 1.2864e-7;
c_m = k_m/k_f;

M_o = [1 1 1 1 1 1;...
    (1/2)*r r (1/2)*r -(1/2)*r -r -(1/2)*r;...
    -sqrt(3)*r/2 0 sqrt(3)*r/2 sqrt(3)*r/2 0 -sqrt(3)*r/2;...
    -c_m c_m -c_m c_m -c_m c_m];
f_avant = pinv(M_o)*u_avant;

f_apres = zeros(6,1);
for i = 1:6
    f_apres(i,1) = constraint(f_avant(i,1), 2.352, 10.6232); % KV920
end
u_apres = M_o * f_apres;

out1 = u_apres;
out2 = f_apres;
end

