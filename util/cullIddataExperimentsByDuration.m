function data = cullIddataExperimentsByDuration(data, min_duration)
  n_experiments = size(data,4);
  idx_keep = true(1,n_experiments);
  for i = 1:n_experiments
    duration_i = [-1,1]*data(:,:,:,i).SamplingInstants([1;end]);
    idx_keep(i) = duration_i > min_duration;
  end
  data = data(:,:,:,find(idx_keep));
end
