function is_zero = spot_isZero( p )
%SPOT_ISZERO Returns true if the polynomial represented by p is 0
is_zero = (sum(size(p.var)) == 0);
end

