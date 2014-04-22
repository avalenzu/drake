function data = indexIddataByTime(data,tspan)
  if iscell(data)
    for i=1:length(data)
      data{i} = indexIddataByTime(data{i},tspan);
      if ~iscell(data{i})
        data{i} = data(i);
      end
    end
    data = data(cellfun(@(x)~isempty(x),data));
    data = reshape([data{:}], ...
                   length(data{1}),length(data));
  else
    typecheck(data,'iddata');
    n_segments = size(tspan,1); 
    if all(size(tspan) > 1)
      data_cell = cell(1,n_segments);
      for i = 1:n_segments
        data_cell{i} = indexIddataByTime(data,tspan(i,:));
      end
     %data = merge(data_cell{cellfun(@(x)~isempty(x),data_cell)});
     data = data_cell(cellfun(@(x)~isempty(x),data_cell));
    else
      idx_within_tspan = tspan(1) <= data.SamplingInstants ...
        & data.SamplingInstants<=tspan(end);
      if any(idx_within_tspan)
        data = data(idx_within_tspan);
      else
        data = {};
      end
    end
  end
end
