function output = constraint(val,min_val,max_val)
if val > max_val
    output = max_val;
elseif val < min_val
    output = min_val;
else
    output = val;
end
end