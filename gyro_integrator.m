clear all;
close all;

%load 20200828_080644.mat;
%a = [AX AY AZ];
%w = [WX WY WZ];

load yrotation.mat
w= [wx wy wz];

fs = 100;
dt = 1/fs;
[N,M] = size(w);

R(1:3,1:3,1)=eye(3);

figure(1);
axis([-1  1 -1 1 -1 1]); hold on; grid on;
view(3);
quiver3(0,0,0,R(1,1,1),R(2,1,1),R(3,1,1),'r');
quiver3(0,0,0,R(1,2,1),R(2,2,1),R(3,2,1),'g');
quiver3(0,0,0,R(1,3,1),R(2,3,1),R(3,3,1),'b');
hold off;
pause();
clf;
for i = 2:N
	S = skewSymMat(w(i-1,:));
	R(:,:,i) = R(:,:,i-1)*expM(dt*S);
		figure(1);
		clf;
		axis([-1  1 -1 1 -1 1]); hold on; grid on;
		view(3);
		quiver3(0,0,0,R(1,1,i),R(2,1,i),R(3,1,i),'r');
		quiver3(0,0,0,R(1,2,i),R(2,2,i),R(3,2,i),'g');
		quiver3(0,0,0,R(1,3,i),R(2,3,i),R(3,3,i),'b');
		hold off;
		pause(dt);
%		clf;
end




