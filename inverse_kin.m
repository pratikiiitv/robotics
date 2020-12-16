clear all;
close all;

l = [10; 5];
theta = [pi/6; pi/3];
x = para2conf(l,theta);

% Newton Raphson Iterative Solver

T = [0;0];
for iter = 1:100
df = [l(1,1)*sin(T(1,1))+l(2,1)*sin(T(1,1)+T(2,1)) l(2,1)*sin(T(1,1)+T(2,1));
-l(1,1)*cos(T(1,1))-l(2,1)*cos(T(1,1)+T(2,1)) -l(2,1)*cos(T(1,1)+T(2,1))];
f = x - para2conf(l,T);
T = T - df\f; % inv[df]*f
end

