clear all;
close all;

N = 100;
c = 2;
m = 1;
w = .1;
%% 1-(1-w^2)^num_iter = .9
p = 0.9;
num_iter = ceil(log(1-p)/log(1-w^2));

x = randn(1,N);
y = c + m*x + 0.5*randn(1,N);


T = 1;

Rat = 0;

%figure(1);
%plot(x,y,'.'); hold on;

for n = 1:num_iter

	i = randperm(N,2);
	# Expression of m and c in terms of two points (x(i),y(i))
	m_ = (y(i(2))-y(i(1)))/(x(i(2))-x(i(1)));	
	c_ = y(i(1))-m_*x(i(1));
	y1 = m_*x+c_;
	e = abs(y-y1);
	Rat_ = sum(e<=T)/N
%	plot(x,y1,'.r'); grid on;
%	plot(x(i(1)),y(i(1)),'+k');
%	plot(x(i(2)),y(i(2)),'+k');
%	pause();
	if Rat_ > Rat
		m_hat = m_;
		c_hat = c_;
		Rat = Rat_;
	end 
end

y1 = m_hat*x+c_hat;

figure(1);
plot(x,y,'.'); hold on;
plot(x,y1,'.r'); grid on;
plot(x(i(1)),y(i(1)),'+k');
plot(x(i(2)),y(i(2)),'+k');

