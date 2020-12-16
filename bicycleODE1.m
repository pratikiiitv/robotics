function dydt = bicycleODE1(t, y, gam, v, L, xg, yg, kv, kgam)

% Reaching a Point
% y = [x y theta]

v = kv*sqrt((xg-y(1))^2+(yg-y(2))^2);
theta = atan2(yg-y(2),xg-y(1));
gam = kgam*(theta - y(3));

dydt = [v*cos(y(3)); v*sin(y(3)); v*tan(gam)/L];


