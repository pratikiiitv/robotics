% -------------------------------------------------------
%
% Author : Pratik Shah
% Date : October 2020
% Place : Gandhinagar
% 
% Extended Kalman Filter for Localization of bicycle type 
% four wheeled robot with noisy odometry based on 
% range and bearing sensors.
%
% Reference: Robotics, Vision and Control Fundamental 
% Algorithms in Matlab by Peter Corke (Chapter 6)
%
% For course on Introduction to Robotics
% Autumn 2020-21 CS427 3-0-2-4
% Indian Institute of Information Technology Vadodara
%
%--------------------------------------------------------

clear all;
close all;

v_max = 1; % 1 m/sec
steer_max = 0.5; % 0.5 degrees/sec
dt = 0.1; % sec
s_d = .02; % meters per sample interval
s_theta = 0.5*pi/180; %radians per sample interval
odo_noise = [s_d^2 0; 0 s_theta^2];

% ------- True state ----
x(:,1) = [0;0;0]; % [x,y,theta]

% ------- Odometry corrupted state -----
X = x; X_ = x;
P(:,:,1) = diag([0.005 0.005 0.001].^2);
detPxy(1) = sqrt(det(P(:,:,1)));
detPxy_ = detPxy;

% ------- Deployed Landmark - MAP -------
N = 10; # Number of landmark points
landmark = 10*rand(2,N);

% ------- Landmark sensing noise -------
sigmar = 0.1; sigmab =1*pi/180;
W = diag([sigmar^2 sigmab^2]);

T = 1000;
turn = 1;

for k = 1:T

	v(k) = 1; steer(k) = 0.3;
	if mod(k,200)==0
		% ------- Input -------
		turn = -1*turn;;
	endif
	
	delta_actual(:,k) = [v(k)*dt;turn*steer(k)*dt]; 

	% ------- Odometry ------
	delta_odo(:,k) = delta_actual(:,k)+sqrt(odo_noise)*randn(2,1);
	
	% ------- Actual State ------
	actual_update = [delta_actual(1,k)*cos(x(3,k));
		delta_actual(1,k)*sin(x(3,k));
		delta_actual(2,k)];
	x(:,k+1) = x(:,k) + actual_update;

	% ------- Odo state ------
	odo_update = [delta_odo(1,k)*cos(x(3,k));
		delta_odo(1,k)*sin(x(3,k));
		delta_odo(2,k)];
	X(:,k+1) = X(:,k) + odo_update;
	
	% ------- Predicted State Covariance --------
	Fx = [1 0 -delta_odo(1,k)*sin(X(3,k));
		0 1 delta_odo(1,k)*cos(X(3,k));
		0 0 1];
	Fv = [cos(X(3,k)) 0;
		sin(X(3,k)) 0;
		0 1];
	P(:,:,k+1) = Fx*P(:,:,k)*Fx'+Fv*odo_noise*Fv';

%
	% Landmark sensing
	landmarkIndex = randperm(N,1); # select the landmark index for measuring range and bearing
	z = [norm(landmark(:,landmarkIndex)-x(1:2,k+1));
	atan((landmark(2,landmarkIndex)-x(2,k+1))/(landmark(1,landmarkIndex)-x(1,k+1)))-x(3,k+1)]+sqrt(W)*randn(2,1);	
	
	% Calculate innovation
	dY = landmark(2,landmarkIndex)-X(2,k+1);
	dX = landmark(1,landmarkIndex)-X(1,k+1);
	r = sqrt(dX^2+dY^2);
	v = z - [r; atan(dY/dX)-X(3,k+1)];

	% Kalman Gain calculation
	Hx = [-dX/r -dY/r 0;dY/r^2 -dX/r^2 -1];
	Hw = [1 0;0 1];
	S = Hx * P(:,:,k+1) * Hx' + Hw * W * Hw';
	K = P(:,:,k+1)*Hx'*inv(S);
	
	% State update equations
	X(:,k+1) = X(:,k+1) + K * v;
	P(:,:,k+1) = P(:,:,k+1) - K*Hx*P(:,:,k+1);
%}
	detPxy(k+1) = sqrt(det(P(:,:,k+1)));
end

figure(1);
%axis([-10 10 -10 10]); hold on; grid on;
subplot(2,1,1);
plot(x(1,:),x(2,:),'k');
hold on;
plot(X(1,:),X(2,:),'r');
grid on;
subplot(2,1,2);
plot(detPxy);
