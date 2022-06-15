function [p] = evalLine(l, t)

% Get a point for the given t on line l
x = l(1,1) + t*l(1,2);
y = l(2,1) + t*l(2,2);
p = [x, y];

end

