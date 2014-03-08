function data = resampleIddataWithTs(data,Ts,varargin)
  typecheck(data,'iddata');
  n_experiments = size(data,4);
  if n_experiments > 1
    data_cell = cell(1,n_experiments);
    for i = 1:n_experiments
      [q,p] = rat(Ts/data.Ts{i});
      data_cell{i} = resample(data(:,:,:,i),p,q,varargin{:});
    end
    data = merge(data_cell{:});
  else
    [q,p] = rat(Ts/data.Ts);
    data = resample(data,p,q,varargin{:});
  end
  %data_cell{i} = data_cell{i}(:);
end
