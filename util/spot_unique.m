function [ p, ia, ic ] = spot_unique( p )
%SPOT_UNIQUE Returns the unique elements of p as a column vector
if length(p) > 0
  p = reshape(p,[],1);
  i = 1; j = 2;
  ia = 1:length(p);
  ic = 1:length(p);
  
  while i < length(ia)
    j = i+1;
    while j <= length(ia)
      if spot_isequal(p(ia(i)),p(ia(j)))
        ia(j) = [];
        ic(j) = ia(i);
        ic(j+1:end) = ic(j+1:end) - 1;
      else
        j = j+1;
      end
    end
    i = i+1;
  end
  p = p(ia);
else
  ia = []; ic = [];
end
