function [x] = para2conf(l,theta)

x(1,1) = l(1,1)*cos(theta(1,1))+l(2,1)*cos(theta(1,1)+theta(2,1));
x(2,1) = l(1,1)*sin(theta(1,1))+l(2,1)*sin(theta(1,1)+theta(2,1));
