%------------------------------------------------------------
% Author : Pratik Shah
% Date : Aut 20, 2020
% Place : Gandhinagar, India  
% Bicycle type four wheel robot kinematic simulation
% x' = v cos(theta)
% y' = v sin(theta)
% theta' = v/L tan(gamma)
% Following a trajectory (xg(t),yg(t))
%------------------------------------------------------------

clear all;
close all;

L = 1; % Wheel base - 1 meter

TS = 0;
TE = 30; % Duration of simulation
tspan = [TS TE]; % Time interval for simulation


gam = 0; % initial steering angle
v = 0; % initial speed

% Initial state of the vehicle
x0 = 0;
y0 = 0;
theta0 = pi/2;
Y0 = [x0;y0;theta0];

y(:,1) = [x0; y0; theta0];
if (y(3,1) >= pi)
	y(3,1) = y(3,1) - 2*pi;
elseif (y(3,1) < -pi)
	y(3,1) = y(3,1) + 2*pi;
end	

% Proportional Control
kv = 4; ki = 1; kgam = 2;

% Simple Integrator
dt = 0.01;
%idx = 1;
d = .5;

%while (norm(y(1:2,idx)-[xg(idx);yg(idx)]) > .01)
t = TS:dt:TE;
xg = (4-3*exp(-t)).*cos(2*pi*.1*t); yg = 1+(4-3*exp(-t)).*sin(2*pi*.1*t);

%figure(1);
%subplot(2,2,1);
%axis([-7 7 -7 7]);
%grid on; hold on;

E = 0;

for i = 1:length(t)

	e(i) = sqrt((xg(i)-y(1,i))^2+(yg(i)-y(2,i))^2)-d;
	E = E+e(i);
	EE(i) = E;
	v(i) = kv*e(i)+ki*dt*E;
	v(i) = conditioning1(v, i, dt, 0, 3, 0.2);	
	
	theta(i) = atan2(yg(i)-y(2,i),xg(i)-y(1,i));

	if (i==1)
		dtheta(i) = theta(i) - y(3,i);
	else
		dtheta = theta(i) - y(3,i);
		if (dtheta <= -pi)
			dtheta = 2*pi + dtheta;
		elseif (dtheta > pi )
			dtheta = -2*pi +dtheta;
		endif
		dtheta(i) = dtheta;
	endif	

	gam(i) = kgam*dtheta(i);
	gam(i) = conditioning1(gam, i, dt, -pi/4, pi/4,pi/10);

	dydt = [v(i)*cos(y(3,i)); v(i)*sin(y(3,i)); v(i)*tan(gam(i))/L];
	
	y(:,i+1) = y(:,i)+dt*dydt;

	if (y(3,i+1) > pi)
		y(3,i+1) = y(3,i+1) - 2*pi;
	elseif (y(3,i+1) <= -pi)
		y(3,i+1) = y(3,i+1) + 2*pi;
	endif	

%	plot(xg(i),yg(i),'b+'); 
%	plot(y(1,i),y(2,i),'r');
%	pause(.001); 
end

figure(1);
subplot(2,2,1);
axis([-7 7 -7 7]); hold on;
plot(xg,yg,'b+'); 
plot(y(1,:),y(2,:),'r.');
grid on; 
subplot(2,2,2);
grid on; hold on;	
plot(t,y(3,1:end-1),'r'); 
plot(t,theta);	
subplot(2,2,3);
plot(t,v,'r'); grid on; hold on;
plot(t,dt*EE,'k'); plot(t,e);
subplot(2,2,4);
plot(t,gam); grid on;
hold on; plot(t,dtheta);


