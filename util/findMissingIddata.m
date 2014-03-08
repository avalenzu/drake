function idx_missing = findMissingIddata(data)
  typecheck(data,'iddata');
  idx_missing = any(isnan(data.OutputData),2);
end
