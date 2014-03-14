function data = quatrotateIddata(quat_data,vec_data,output_name)
  if size(quat_data,4) ~= size(vec_data,4)
    error('quatrotateIddata:NumExperiments', ...
          'Arguments must have the same number of experiments');
  elseif any(size(quat_data,1) ~= size(vec_data,1));
    error('quatrotateIddata:NumSamples', ...
          'Arguments must contain the same number of samples for each experiment');
  elseif iscell(quat_data.Ts) && any(cell2mat(quat_data.Ts) ~= cell2mat(vec_data.Ts));
    error('quatrotateIddata:SamplingTime', ...
          'Arguments must have the same samplng time for each experiments');
  elseif size(quat_data,2) ~= 4 || size(vec_data,2) ~= 3
    error('quatrotateIddata:NumOutputs', ...
          'Arguments must contain the 4 and 3 outputs respectively');
  end
  n_experiments = size(quat_data,4);
  data_cell = cell(n_experiments,1);
  for i = 1:n_experiments
    r_rotated_data = quatrotate(quat_data(:,:,:,i).OutputData,vec_data(:,:,:,i).OutputData);
    data_cell{i} = DrakeIddata(r_rotated_data,[],quat_data.Ts(i));
  end
  data = merge(data_cell{:});
  if nargin >= 3
    data = DrakeIddata(data.OutputData,[],data.Ts,'OutputName',cellStrCat(output_name,'_',{'x','y','z'}));
  end
end
