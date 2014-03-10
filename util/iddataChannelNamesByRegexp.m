function matching_names = iddataChannelNamesByRegexp(data,expr)
  % matching_names = iddataChannelNamesByRegexp(data, expr) - Returns a
  % cell array of strings containing the names of the channels in `data`
  % that match the regular expression `expr`.
  %
  % @param data - iddata object
  % @param expr - String containing a regular expression
  % @retval matching_names - Cell array of channel names that match expr
  %

  matching_names = regexp(data.OutputName,expr,'match');
  matching_names(cellfun('isempty',matching_names)) = [];
end
