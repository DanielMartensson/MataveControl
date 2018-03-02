% Generates the low frequency gain of a state space model or a transfer function
% Input: sys, G
% Example 1: dc = dcgain(sys)
% Example 2: dc = dcgain(G)
% Author: Daniel Mårtensson 2017 September
% Updated 2018 Mars

function [dc] = dcgain(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error ('Missing input')
  end
  
  % Get the type
  type = varargin{1}.type;
  % Check if there is a TF or SS model
  if(strcmp(type, 'SS' ))
    % Get necessary info 
    A = varargin{1}.A;
    B = varargin{1}.B;
    C = varargin{1}.C;
    D = varargin{1}.D;
    dc = C*inv(-A)*B + D;
  elseif(strcmp(type, 'TF' ))
    % Get necessary info 
    for i = 1:size(varargin{1},1)
      for j = 1:size(varargin{1},2)
        % Get necessary info 
        G = varargin{1}(i, j);
        % Get the static gain
        % But if G.den(length(G.den)) = 0, then we say G.den(length(G.den)) = 1
        % Because num can be 1 2 3 1 and den can be 1 2 0 0
        % But if G.num(length(G.num)) = 0, then dc gain will be 0
        if G.den(length(G.den)) == 0
          dc(i, j) = G.num(length(G.num))/1;
        else
          dc(i, j) = G.num(length(G.num))/G.den(length(G.den));
        end
        
        % If divided by zero - Is not a number
        if isnan(dc(i, j))
          disp(sprintf('Divided my zero - dcgain (%i, %i) set to 0', i, j))
          dc(i, j) = 0;
        end
      end
    end
  else
    error('This is not TF or SS');
  end
end
