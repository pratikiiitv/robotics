clear all;
close all;

%load 20200828_080644.mat;
%AX = AX-0.55;
%a = [AX AY AZ];
%a = a';
%w = [WX WY WZ];
%w = w';

%load square_room_data.mat

%a = [ax ay az]; w = [wx wy wz];
%[N,M] = size(w);

N = 1000;
Ax = zeros(N,1); Ay = zeros(N,1); Az = 9.8*ones(N,1);
Ay(1:50,1)=20;Ay(51:100,1)=-20;Ay(201:250,1)=20;Ay(251:300,1)=-20;Ay(401:450,1)=20; Ay(451:500,1)=-20;Ay(601:650,1)=20;Ay(651:700,1)=-20;Ay(801:850,1)=20; Ay(851:900,1)=-20;
Wx = zeros(N,1); Wz = zeros(N,1); Wz(100:200,1)=pi/2; Wz(300:400,1)=pi/2; Wz(500:600,1)=pi/2; Wz(700:800,1)=pi/2;Wz(900:1000,1)=pi/2;Wy = zeros(N,1);
w = [Wx Wy Wz];
a = [Ax Ay Az];
a = a'; w = w';

fs = 100;
dt = 1/fs;

R(1:3,1:3,1) = eye(3);
v(1:3,1) = 0;
g(:,1) = [0;0;9.8];
p(1:3,1) = 0;

figure(1);
axis([-6  6 -6 6 -6 6]); hold on; grid on;
view(3);
quiver3(0,0,0,R(1,1,1),R(2,1,1),R(3,1,1),'r');
quiver3(0,0,0,R(1,2,1),R(2,2,1),R(3,2,1),'g');
quiver3(0,0,0,R(1,3,1),R(2,3,1),R(3,3,1),'b');
%hold off;
pause();
%clf;

for i = 2:N
	R(:,:,i) = R(:,:,i-1)*expM(dt*skewSymMat(w(:,i-1)));
	v(:,i) = v(:,i-1)+dt*(R(:,:,i-1)*a(:,i-1)-g);
	p(:,i) = p(:,i-1)+dt*v(:,i-1);
	if mod(i,20)==0	
%		figure(1);
%		clf;
%		axis([-6  6 -6 6 -6 6]); hold on; grid on;
		view(3);
		quiver3(p(1,i),p(2,i),p(3,i),R(1,1,i),R(2,1,i),R(3,1,i),'r');
		quiver3(p(1,i),p(2,i),p(3,i),R(1,2,i),R(2,2,i),R(3,2,i),'g');
		quiver3(p(1,i),p(2,i),p(3,i),R(1,3,i),R(2,3,i),R(3,3,i),'b');
%		hold off;
		pause(dt);
	end
end
