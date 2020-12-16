%------------------------------------------------------------
% Author : Pratik Shah
% Date : Aut 16, 2020
% Place : Gandhinagar, India  
% Bicycle type four wheel robot kinematic simulation
% x' = v cos(theta)
% y' = v sin(theta)
% theta' = v/L tan(gamma)
%------------------------------------------------------------

clear all;
close all;

L = 1; % Wheel base - 1 meter

TS = 0;
TE = 10; % Duration of simulation
tspan = [TS TE] % Time interval for simulation

% Input speed and steering angle gamma
v = ones(size(tspan));
tv = tspan;
gam = [0 0 0 0 .5 0 -.5 0 0 0 0];
tgam = [0:10];

% Initial state of the vehicle
x0 = 0;
y0 = 0;
theta0 = 0;
Y0 = [x0;y0;theta0];

% Setup the Ordinary Differential Equation and solve it (integrate) using ODE45
opts=odeset("MaxStep",0.01);
[t,y] = ode45(@(t,y) bicycleODE(t, y, gam, tgam, v, tv, L), tspan, Y0, opts);


