% -------------------------------------------------------
%
% Author : Pratik Shah
% Date : October 2020
% Place : Gandhinagar
% 
% Particle Filter for Localization of bicycle type 
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

% ------- Deployed Landmark - MAP -------
N = 10; # Number of landmark points
landmark = 10*rand(2,N);

% ------- Landmark sensing noise -------
sigmar = 0.1; sigmab =1*pi/180;
W = diag([sigmar^2 sigmab^2]);

% ------- Intialization of Particle Filter -------
n = 100; % Number of particles
X = diag([5, 5, 2*pi])*rand(3,n);
L = diag([0.1 0.1]);
pert = diag([.02 .02 .5*pi/180]);

%----------------------------------------------
% Importance sampler or resampling
function [Rid] = impSampler(Xweight, n)
% n = number of samples to be drawn in Rid

cumw = cumsum(Xweight);
for i = 1:n
	k = rand; j = 1;
	while cumw(j)<k
		j = j+1;
	end
	Rid(i) = j;
end

end
%---------------------------------------------

% ------- Simulation time line -------
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

	% ------- Particles updated with odometry ------
	odo_update = [delta_odo(1,k)*cos(x(3,k));
		delta_odo(1,k)*sin(x(3,k));
		delta_odo(2,k)];
	X(1,:) = X(1,:) + odo_update(1);
	X(2,:) = X(2,:) + odo_update(2);
	X(3,:) = X(3,:) + odo_update(3);
%
	% Landmark sensing
	landmarkIndex = randperm(N,1); # select the landmark index for measuring range and bearing
	z = [norm(landmark(:,landmarkIndex)-x(1:2,k+1));
	atan((landmark(2,landmarkIndex)-x(2,k+1))/(landmark(1,landmarkIndex)-x(1,k+1)))-x(3,k+1)]+sqrt(W)*randn(2,1); % Measurement	
	
	% Calculate innovation
	dY = landmark(2,landmarkIndex)-X(2,:);
	dX = landmark(1,landmarkIndex)-X(1,:);
	r = sqrt(dX.^2+dY.^2);
	V(1,:) = z(1)-r(1,:); 
	V(2,:) = atan(dY./dX)-X(3,:);

	% Assigning Weights to particles
	Xweight = exp(-V'*L*V)+1e-5;
	Xweight = Xweight/sum(Xweight);

	% Resampling the particles
	Rid = impSampler(Xweight, n, n);
	X = X(:,Rid) + pert*randn(3,n);

	% Calculate the covariance
	Xm = mean(X,2);
	CX = (X-Xm)*(X-Xm)'/n;
	s(1,k) = sqrt(CX(1,1));
	s(2,k) = sqrt(CX(2,2));
	s(3,k) = sqrt(CX(3,3));	
	
%	disp(k);	

%}
end


