clear all;
close all;

L1 = 5; L2 = 3; L3 = 2;
T1 = pi/2; T2 = 0; T3 = 0;

x = L1*cos(T1)+L2*cos(T1+T2)+L3*cos(T1+T2+T3);
y = L1*sin(T1)+L2*sin(T1+T2)+L3*sin(T1+T2+T3);
phi = mod(T1+T2+T3,2*pi);

% Newton Raphson Iterative Solver

theta1 = pi/4; theta2 = 0; theta3 = 0;
epsilon = 100;

while(epsilon>=.001)

x_ = L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3);
y_ = L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3);
phi_ = theta1+theta2+theta3;
phi_ = mod(phi_,2*pi);
f = [x;y;phi] - [x_;y_;phi_];

dfx = [L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3); L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3); L3*sin(theta1+theta2+theta3)];

dfy = -[(L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)); (L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)); (L3*cos(theta1+theta2+theta3))];

dfphi = -[1;1;1];
df = [dfx'; dfy';dfphi'];

Theta = [theta1; theta2; theta3];
Theta = Theta - df\f; 
Theta = mod(Theta,2*pi);
theta1 = Theta(1);
theta2 = Theta(2);
theta3 = Theta(3);

epsilon = norm(f);
disp(epsilon); pause();

end

