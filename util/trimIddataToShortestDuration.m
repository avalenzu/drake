function data_cell = trimIddataToShortestDuration(data_cell)
  t_0 = max(cellfun(@(data) data.SamplingInstants(1),data_cell));
  t_f = min(cellfun(@(data) data.SamplingInstants(end),data_cell));
  data_cell = cellfun(@(data) indexIddataByTime(data,[t_0,t_f]), ...
                      data_cell, 'UniformOutput',false);
end
