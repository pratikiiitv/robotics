function q = conditioning(qnow, qold, dt, qmin, qmax)%, qqmin, qqmax)


% Speed limit
if qnow < qmin
	q = qmin;
elseif qnow > qmax
	q = qmax;
else
	q = qnow; 
end

% Acceleration (rise/ fall) limit



