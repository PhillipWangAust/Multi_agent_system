function [y] = sig(x,a)
y = sign(x)*power(abs(x),a);
end

