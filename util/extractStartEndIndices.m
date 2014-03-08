function start_end_indices = extractStartEndIndices(logical_vec)
  % @param logical_vec - Logical vector 
  % @retval start_end_indices - 2 x n segments array where the i-th row
  % consists of the starting index and ending index of the i-th segment
  % contiguous 'true' values in logical_vec.
  logical_vec = reshape(logical_vec,[],1);
  start_indices = find(logical_vec(1:end) & ~[false;logical_vec(1:end-1)]);
  end_indices   = find(logical_vec(1:end) & ~[logical_vec(2:end);false]);
  start_end_indices = [start_indices,end_indices];
end
