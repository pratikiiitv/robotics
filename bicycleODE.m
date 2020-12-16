function dydt = bicycleODE(t,y, gam, tgam, v, tv, L)

% y = [x y theta]

v = interp1(tv, v, t);
gam = interp1(tgam, gam, t);

dydt = [v.*cos(y(3)); v.*sin(y(3)); v.*tan(gam)/L];


