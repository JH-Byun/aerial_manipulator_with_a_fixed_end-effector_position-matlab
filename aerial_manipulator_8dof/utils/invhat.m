function out = invhat(mat)
mat_size = size(mat);

out(1,1) = mat(3,2);
out(2,1) = mat(1,3);
out(3,1) = mat(2,1);

end