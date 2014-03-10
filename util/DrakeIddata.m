classdef DrakeIddata
  % This class exists to allow extension of MATLAB's builtin iddata
  % class. Unfortunately as of R2012b that class is still an old-style
  % class, so it can't be subclassed by a new-style class. This class
  % implements passthroughs for all the methods listed by 
  %
  % >> methods(iddata)
  %
  properties
    data;
  end
  methods
    function obj = DrakeIddata(varargin)
      if nargin == 1
        typecheck(varargin{1},'iddata');
        obj.data = varargin{1};
      else
        obj.data = iddata(varargin{:});
      end
    end

    function matching_names = iddataChannelNamesByRegexp(obj,expr)
      % matching_names = iddataChannelNamesByRegexp(data, expr) - Returns a
      % cell array of strings containing the names of the channels in `data`
      % that match the regular expression `expr`.
      %
      % @param data - iddata object
      % @param expr - String containing a regular expression
      % @retval matching_names - Cell array of channel names that match expr
      %

      matching_names = regexp(obj.data.OutputName,expr,'match','once');
      matching_names = obj.data.OutputName(strcmp(matching_names,obj.data.OutputName));
    end

    function varargout = subsref(obj,S)
      for i = 1:length(S)
        if iscell(S(i).subs)
          if iscell(S(i).subs{2})
            name_cell = {};
            for i = 1:length(S(i).subs{2})
              name_cell = [name_cell; iddataChannelNamesByRegexp(obj,S(i).subs{2}{i})];
            end
            S(i).subs{2} = unique(name_cell,'stable');
          elseif ischar(S(i).subs{2})
            S(i).subs{2} = iddataChannelNamesByRegexp(obj,S(i).subs{2});
          end
        end
      end
      [varargout{1:nargout}] = subsref(obj.data,S);

    end

    function ind = end(obj,k,n)
      szd = size(obj.data);
      if k <= n
        ind = szd(k);
      else
        error('DrakeIddata:end','k must be less than or equal to n');
      end
    end

    function data = merge(obj,varargin)
      data_cell = [{obj.data},cellfun(@(x)x.data,varargin,'UniformOutput',false)];
      data = DrakeIddata(merge(data_cell{:}));
    end

  end

  methods % Pure passthroughs
    function varargout = abs(obj,varargin)
      [varargout{1:nargout}] = abs(obj.data,varargin{:});
    end

    function varargout = complex(obj,varargin)
      [varargout{1:nargout}] = complex(obj.data,varargin{:});
    end

    function varargout = display(obj,varargin)
      [varargout{1:nargout}] = display(obj.data,varargin{:});
    end

    function varargout = fcat(obj,varargin)
      [varargout{1:nargout}] = fcat(obj.data,varargin{:});
    end

    function varargout = get(obj,varargin)
      [varargout{1:nargout}] = get(obj.data,varargin{:});
    end

    function varargout = getid(obj,varargin)
      [varargout{1:nargout}] = getid(obj.data,varargin{:});
    end

    function varargout = idprep_f(obj,varargin)
      [varargout{1:nargout}] = idprep_f(obj.data,varargin{:});
    end

    function varargout = isaimp(obj,varargin)
      [varargout{1:nargout}] = isaimp(obj.data,varargin{:});
    end

    function varargout = isreal(obj,varargin)
      [varargout{1:nargout}] = isreal(obj.data,varargin{:});
    end

    function varargout = pexcit(obj,varargin)
      [varargout{1:nargout}] = pexcit(obj.data,varargin{:});
    end

    function varargout = pvget(obj,varargin)
      [varargout{1:nargout}] = pvget(obj.data,varargin{:});
    end

    function varargout = resample(obj,varargin)
      [varargout{1:nargout}] = resample(obj.data,varargin{:});
    end

    function varargout = setid(obj,varargin)
      [varargout{1:nargout}] = setid(obj.data,varargin{:});
    end

    function varargout = uset(obj,varargin)
      [varargout{1:nargout}] = uset(obj.data,varargin{:});
    end

    function varargout = advice(obj,varargin)
      [varargout{1:nargout}] = advice(obj.data,varargin{:});
    end

    function varargout = convertToIdfrd(obj,varargin)
      [varargout{1:nargout}] = convertToIdfrd(obj.data,varargin{:});
    end

    function varargout = dtrend(obj,varargin)
      [varargout{1:nargout}] = dtrend(obj.data,varargin{:});
    end

    function varargout = feedback(obj,varargin)
      [varargout{1:nargout}] = feedback(obj.data,varargin{:});
    end

    function varargout = getMetaData(obj,varargin)
      [varargout{1:nargout}] = getMetaData(obj.data,varargin{:});
    end

    function varargout = greyest(obj,varargin)
      [varargout{1:nargout}] = greyest(obj.data,varargin{:});
    end

    function varargout = idprep_fp(obj,varargin)
      [varargout{1:nargout}] = idprep_fp(obj.data,varargin{:});
    end

    function varargout = isempty(obj,varargin)
      [varargout{1:nargout}] = isempty(obj.data,varargin{:});
    end

    function varargout = llset(obj,varargin)
      [varargout{1:nargout}] = llset(obj.data,varargin{:});
    end

    function varargout = phase(obj,varargin)
      [varargout{1:nargout}] = phase(obj.data,varargin{:});
    end

    function varargout = pvset(obj,varargin)
      [varargout{1:nargout}] = pvset(obj.data,varargin{:});
    end

    function varargout = retrend(obj,varargin)
      [varargout{1:nargout}] = retrend(obj.data,varargin{:});
    end

    function varargout = sim(obj,varargin)
      [varargout{1:nargout}] = sim(obj.data,varargin{:});
    end

    function varargout = timemark(obj,varargin)
      [varargout{1:nargout}] = timemark(obj.data,varargin{:});
    end

    function varargout = vertcat(obj,varargin)
      [varargout{1:nargout}] = vertcat(obj.data,varargin{:});
    end

    function varargout = angle(obj,varargin)
      [varargout{1:nargout}] = angle(obj.data,varargin{:});
    end

    function varargout = covf(obj,varargin)
      [varargout{1:nargout}] = covf(obj.data,varargin{:});
    end

    function varargout = dunique(obj,varargin)
      [varargout{1:nargout}] = dunique(obj.data,varargin{:});
    end

    function varargout = fft(obj,varargin)
      [varargout{1:nargout}] = fft(obj.data,varargin{:});
    end

    function varargout = getTrend(obj,varargin)
      [varargout{1:nargout}] = getTrend(obj.data,varargin{:});
    end

    function varargout = horzcat(obj,varargin)
      [varargout{1:nargout}] = horzcat(obj.data,varargin{:});
    end

    function varargout = ifft(obj,varargin)
      [varargout{1:nargout}] = ifft(obj.data,varargin{:});
    end

    function varargout = isinf(obj,varargin)
      [varargout{1:nargout}] = isinf(obj.data,varargin{:});
    end

    function varargout = plot(obj,varargin)
      [varargout{1:nargout}] = plot(obj.data,varargin{:});
    end

    function varargout = real(obj,varargin)
      [varargout{1:nargout}] = real(obj.data,varargin{:});
    end

    function varargout = rmzero(obj,varargin)
      [varargout{1:nargout}] = rmzero(obj.data,varargin{:});
    end

    function varargout = size(obj,varargin)
      [varargout{1:nargout}] = size(obj.data,varargin{:});
    end

    function varargout = detrend(obj,varargin)
      [varargout{1:nargout}] = detrend(obj.data,varargin{:});
    end

    function varargout = fieldnames(obj,varargin)
      [varargout{1:nargout}] = fieldnames(obj.data,varargin{:});
    end

    function varargout = iddata(obj,varargin)
      [varargout{1:nargout}] = iddata(obj.data,varargin{:});
    end

    function varargout = imag(obj,varargin)
      [varargout{1:nargout}] = imag(obj.data,varargin{:});
    end

    function varargout = isnan(obj,varargin)
      [varargout{1:nargout}] = isnan(obj.data,varargin{:});
    end

    function varargout = realdata(obj,varargin)
      [varargout{1:nargout}] = realdata(obj.data,varargin{:});
    end

    function varargout = set(obj,varargin)
      [varargout{1:nargout}] = set(obj.data,varargin{:});
    end

    function varargout = step(obj,varargin)
      [varargout{1:nargout}] = step(obj.data,varargin{:});
    end

    function varargout = unpack(obj,varargin)
      [varargout{1:nargout}] = unpack(obj.data,varargin{:});
    end

    function varargout = diff(obj,varargin)
      [varargout{1:nargout}] = diff(obj.data,varargin{:});
    end

    function varargout = estdatch(obj,varargin)
      [varargout{1:nargout}] = estdatch(obj.data,varargin{:});
    end

    function varargout = findstates(obj,varargin)
      [varargout{1:nargout}] = findstates(obj.data,varargin{:});
    end

    function varargout = getexp(obj,varargin)
      [varargout{1:nargout}] = getexp(obj.data,varargin{:});
    end

    function varargout = impulse(obj,varargin)
      [varargout{1:nargout}] = impulse(obj.data,varargin{:});
    end

    function varargout = isnlarx(obj,varargin)
      [varargout{1:nargout}] = isnlarx(obj.data,varargin{:});
    end

    function varargout = numel(obj,varargin)
      [varargout{1:nargout}] = numel(obj.data,varargin{:});
    end

    function varargout = reminf(obj,varargin)
      [varargout{1:nargout}] = reminf(obj.data,varargin{:});
    end

    function varargout = setexp(obj,varargin)
      [varargout{1:nargout}] = setexp(obj.data,varargin{:});
    end

    function varargout = subsasgn(obj,varargin)
      [varargout{1:nargout}] = subsasgn(obj.data,varargin{:});
    end

    function varargout = unwrap(obj,varargin)
      [varargout{1:nargout}] = unwrap(obj.data,varargin{:});
    end
  end
end
