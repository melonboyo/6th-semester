function [t, d] = closestT(l,p)

% Set up a system of equations which will yield the value t for the closest
% point on the line to p.
syms u s;
eq1 = l(1,1)+u*l(1,2) == p(1)+s*-l(2,2);
eq2 = l(2,1)+u*l(2,2) == p(2)+s* l(1,2);
[A,B] = equationsToMatrix([eq1 eq2], [u, s]);
solved = linsolve(double(A),double(B));

t = solved(1);
d = solved(2);

end

