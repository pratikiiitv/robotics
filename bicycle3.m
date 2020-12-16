%------------------------------------------------------------
% Author : Pratik Shah
% Date : Aut 23, 2020
% Place : Gandhinagar, India  
% Bicycle type four wheel robot navigation
% x' = v cos(theta)
% y' = v sin(theta)
% theta' = v/L tan(gamma)
% Reactive Navigation 
% Sensor Field 200./((x-xc).^2+(y-yc).^2+200)
% Move toward the peak of the scalar field
%------------------------------------------------------------

clear all;
close all;

L = 1; % Wheel base - 1 meter

TS = 0;
TE = 100; % Duration of simulation
%tspan = [TS TE]; % Time interval for simulation


gam = 0; % initial steering angle
v = 0; % initial speed

% Initial state of the vehicle
x0 = 0;
y0 = 0;
theta0 = 0;
Y0 = [x0;y0;theta0];

y(:,1) = [x0; y0; theta0];
if (y(3,1) >= pi)
	y(3,1) = y(3,1) - 2*pi;
elseif (y(3,1) < -pi)
	y(3,1) = y(3,1) + 2*pi;
end	

% Proportional Control
kv=1; kgam = 1;

% Simple Integrator
dt = 0.01;

t = TS:dt:TE;

d = 10;

for i = 1:length(t)
	x1(i) = y(1,i)+d*sin(y(3,i)); y1(i) = y(2,i)-d*cos(y(3,i));
	x2(i) = y(1,i)-d*sin(y(3,i)); y2(i) = y(2,i)+d*cos(y(3,i));	
	sr(i) = sensorfield1(x1(i),y1(i));
	sl(i) = sensorfield1(x2(i),y2(i));
	v(i) = 1-sr(i)-sl(i);
	v(i) = conditioning1(v, i, dt, 0, 3, 0.2);	
	
	gam(i) = -100*(sr(i)-sl(i));
	gam(i) = conditioning1(gam, i, dt, -pi/4, pi/4,pi/10);

	dydt = [v(i)*cos(y(3,i)); v(i)*sin(y(3,i)); v(i)*tan(gam(i))/L];
	
	y(:,i+1) = y(:,i)+dt*dydt;

	if (y(3,i+1) > pi)
		y(3,i+1) = y(3,i+1) - 2*pi;
	elseif (y(3,i+1) <= -pi)
		y(3,i+1) = y(3,i+1) + 2*pi;
	endif	
end

A = 1:100;
[a,b] = meshgrid(A,A);
f = sensorfield1(a,b);
figure(1);
subplot(2,2,1); 
contour(a,b,f,100); hold on;
plot(y(1,:),y(2,:),'+r'); grid on;hold off;

subplot(2,2,2);
surf(a,b,f); hold on;
plot3(y(1,:)',y(2,:)',sensorfield1(y(1,:)',y(2,:)'),'y');
plot3(x1',y1',sr,'k+');
plot3(x2',y2',sl,'g+'); grid on;


subplot(2,2,3);
plot(t,v); grid on;

subplot(2,2,4);
plot(t, gam); grid on;

