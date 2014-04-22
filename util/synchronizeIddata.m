function data = synchronizeIddata(data_cell,varargin)
  % Given a 1D cell array of iddata objects each containing a single experiment
  % resample all of the objects at the frequency of the object with the lowest
  % sampling frequency and concatentate the results.
  % Given a 2D cell array apply 1D version to each row and merge the results
  if all(size(data_cell)>1)
    data_cell_out = cell(size(data_cell,1),1);
    for i = 1:size(data_cell,1)
      %data_cell(i,:) = synchronizeIddata(data_cell(i,:));
      data_cell_out{i} = synchronizeIddata(data_cell(i,:));   
    end
    data = merge(data_cell_out{:});
  else
    max_Ts = maxIddataTs(data_cell);
    data_cell_out = cell(size(data_cell));
    for i = 1:length(data_cell)
      data_cell_out{i} = resampleIddataWithTs(data_cell{i},max_Ts);
      %data_cell{i} = data_cell{i}(:);
    end
    t0 = max(cellfun(@(data) data.sa(1),data_cell));
    tf = min(cellfun(@(data) data.sa(end),data_cell));
    t = t0:max_Ts:tf;
    for i = 1:length(data_cell)
      if size(data_cell{i},1) > 1
        data_cell_out{i}.OutputData = interp1(data_cell{i}.sa,data_cell{i}.OutputData,t);
        data_cell_out{i}.SamplingInstants = t-t0;
      else
        data_cell_out{i} = {};
      end
    end
    data = [data_cell_out{cellfun(@(x)~isempty(x),data_cell_out)}];
    %min_nt = min(cellfun(@(data) size(data,1),data_cell));
    %data_cell = cellfun(@(data) data(1:min_nt),data_cell,'UniformOutput',false);
  end
end
