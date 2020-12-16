function [q] = conditioning1(qnow, i, dt, qmin, qmax, qqmax)


% Acceleration (rise/ fall) limit
if i >= 2
	e = qnow(i)-qnow(i-1);
	if abs(e)>qqmax
		q = qnow(i-1)+sign(e)*qqmax;	
	end
end

% Speed limit
if qnow(i) < qmin
	q = qmin;
elseif qnow(i) > qmax
	q = qmax;
else
	q = qnow(i); 
end



