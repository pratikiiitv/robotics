clear all;
close all;

x = input('x');
y = input('y');
phi = input('phi');

function [theta1, theta2, theta3] = myInvKinematics(x,y,phi)

L1 = 5; L2 = 3; L3 = 2;

% --- Write your code here ----
% Check the three link planar open chain inverse kinematics example code
% Read about Newton-Raphson method and implement it 
% The acceptable tolerance is ||(x,y,phi)-(x_,y_,phi_)||<=.1, 
% where x_, y_ and phi_ are the end effector state for given theta values

theta1 = pi/4; theta2 = 0; theta3 = 0; % Initialization
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
Theta = Theta - pinv(df)*f; 
Theta = mod(Theta,2*pi);
theta1 = Theta(1);
theta2 = Theta(2);
theta3 = Theta(3);

epsilon = norm(f);

end

end


[theta1,theta2,theta3] = myInvKinematics(x,y,phi);
theta1
theta2
theta3

% ----


