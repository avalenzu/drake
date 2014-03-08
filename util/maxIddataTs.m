function max_Ts = maxIddataTs(data)
  if iscell(data)
    max_Ts = max(max(cellfun(@maxIddataTs,data)));
  else
    typecheck(data,'iddata');
    max_Ts = data.Ts;
    if iscell(max_Ts)
      max_Ts = max([max_Ts{:}]);
    end
  end
end
  
