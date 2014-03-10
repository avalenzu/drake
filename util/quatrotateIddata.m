function data = quatrotateIddata(obj,quat_str,vec_str,output_name)
  %if size(obj,2) ~= 4
  %error('DrakeIddata:quatrotate:BadInput',...
  %'First argument must have 4 outputs');
  %elseif size(r,2) ~= 3
  %error('DrakeIddata:quatrotate:BadInput',...
  %'Second argument must have 3 outputs');
  %elseif size(
  %end
  n_experiments = size(obj,4);
  data_cell = cell(n_experiments,1);
  for i = 1:n_experiments
    r_rotated_data = quatrotate(obj(:,quat_str,:,i).OutputData,obj(:,vec_str,:,i).OutputData);
    data_cell{i} = DrakeIddata(iddata(r_rotated_data,[],obj.Ts));
  end
  data = merge(data_cell{:});
  if nargin >= 4
    data = DrakeIddata(data.OutputData,[],data.Ts,'OutputName',cellStrCat(output_name,'_',{'x','y','z'}));
  end
end
