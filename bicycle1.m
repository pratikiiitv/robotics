%------------------------------------------------------------
% Author : Pratik Shah
% Date : Aut 16, 2020
% Place : Gandhinagar, India  
% Bicycle type four wheel robot kinematic simulation
% x' = v cos(theta)
% y' = v sin(theta)
% theta' = v/L tan(gamma)
% Reaching a location (xg,yg)
%------------------------------------------------------------

clear all;
close all;

L = 1; % Wheel base - 1 meter

TS = 0;
TE = 10; % Duration of simulation
tspan = [TS TE]; % Time interval for simulation

xg = 5; yg = 5;
gam = 0; % initial steering angle
v = 0; % initial speed

% Initial state of the vehicle
x0 = 2;
y0 = 1;
theta0 = -pi/2;
Y0 = [x0;y0;theta0];

% Integrate the differential equations
y(:,1) = [x0; y0; theta0];

% Proportional Control
kv = .5; kgam = 4;

% Simple Integrator
dt = 0.01;
idx = 1;
while (norm(y(1:2,idx)-[xg;yg]) > .01)
	
%	dydt = bicycle1_dydt(y(:,idx), gam, v, L, xg, yg, kv, kgam);
	v(idx+1) = kv*sqrt((xg-y(1,idx))^2+(yg-y(2,idx))^2);
	v(idx+1) = conditioning(v(idx+1), v(idx), dt, -1, 1);	
	
	theta(idx) = atan2(yg-y(2,idx),xg-y(1,idx));
	gam(idx+1) = kgam*(theta(idx) - y(3,idx));
	gam(idx+1) = conditioning(gam(idx+1), gam(idx), dt, -pi/4, pi/4);

	dydt = [v(idx+1)*cos(y(3,idx)); v(idx+1)*sin(y(3,idx)); v(idx+1)*tan(gam(idx+1))/L];
	
	y(:,idx+1) = y(:,idx)+dt*dydt;
	idx = idx + 1;
end

% Setup the Ordinary Differential Equation and solve it (integrate) using ODE45
%kv = .8; kgam = .7;
%opts=odeset("MaxStep",0.01);
%[t,y] = ode45(@(t,y) bicycleODE1(t, y, gam, v, L, xg, yg, kv, kgam), tspan, Y0, opts);
	
	
	
	



