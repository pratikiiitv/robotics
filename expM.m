function [A,l] = expM(S)

% Input S is a skew symmetric matrix in so(3)
% Output A is a rotation matrix in SO(3)
if S==0
    A = eye(3);
    l = 0;
else
    l = sqrt(S(1,2)^2+S(1,3)^3+S(2,3)^2);
    A = eye(3)+sin(l)/l * S + (1-cos(l))/l^2 * S^2;
end
