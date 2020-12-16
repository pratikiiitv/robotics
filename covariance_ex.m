clear all;
close all;

N = 1000;
x = randn(2,N);
S = [4 0;0 1];
y = S*x;
theta = pi/4;
R = [cos(theta) -sin(theta);sin(theta) cos(theta)];
z = R*y;

mx = mean(x,2);
Cx = (x-mx)*(x-mx)'/N;
my = mean(x,2);
Cy = (y-my)*(y-my)'/N;
mz = mean(z,2);
Cz = (z-mz)*(z-mz)'/N;


figure(1);
subplot(2,2,1);
axis([-15 15 -15 15]); hold on;
plot(x(1,:),x(2,:),'.');
xlabel('x');
ylabel('y');
grid on;
subplot(2,2,2);
axis([-15 15 -15 15]); hold on;
plot(y(1,:),y(2,:),'.');
xlabel('x');
ylabel('y');
grid on;
subplot(2,2,3);
axis([-15 15 -15 15]); hold on;
plot(z(1,:),z(2,:),'.');
xlabel('x');
ylabel('y');
grid on;
