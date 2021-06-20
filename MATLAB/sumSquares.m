function [f] = sumSquares(x,y)

n = length(y);

A = [x(1),x(4),x(5);
    0,x(2),x(6);
    0,0,x(3)];
B = [x(7);x(8);x(9)];

f = zeros(n,1);
for i = 1:n
    ynorm = sqrt(y(1,i)*y(1,i) + y(2,i)*y(2,i) + y(3,i)*y(3,i));
    tmp = A*y(:,i)+B - y(:,i)/ynorm;
    f(i) = sqrt(tmp(1)*tmp(1) + tmp(2)*tmp(2) + tmp(3)*tmp(3));
end