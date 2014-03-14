function data = quatconjIddata(quat_data,output_name)
  if size(quat_data,2) ~= 4
    error('quatrotateIddata:NumOutputs', ...
          'Arguments must contain the 4 and 3 outputs respectively');
  end
  n_experiments = size(quat_data,4);
  data_cell = cell(n_experiments,1);
  for i = 1:n_experiments
    r_rotated_data = quatconj(quat_data(:,:,:,i).OutputData);
    data_cell{i} = DrakeIddata(r_rotated_data,[],quat_data.Ts(i));
  end
  data = merge(data_cell{:});
  if nargin >= 2
    data = DrakeIddata(data.OutputData,[],data.Ts,'OutputName',cellStrCat(output_name,'_',{'x','y','z'}));
  end
end
