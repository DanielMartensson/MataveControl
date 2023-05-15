% Rediscrete transfer function or state space model
% Input: Gd, sysd, sampleTime
% Example 1: G = mc.d2d(Gd, sampleTime)
% Example 2: sys = mc.d2d(sysd, sampleTime)
% Author: Daniel Mårtensson, September 2017

function [model] = d2d(varargin)
  if(isempty(varargin{1}))
    error ('Missing model')
  end
  
  if(isempty(varargin{2}))
    error ('Missing sampeltime')
  end
  
  % State space
  if(strcmp(varargin{1}.type, 'SS' ))
    % To continuous from discrete
    sys = mc.d2c(varargin{1});
    % To discrete from continuous
    model = mc.c2d(sys, varargin{2});
  elseif(strcmp(varargin{1}.type, 'TF' ))
    % To continuous from discrete
    G = mc.d2c(varargin{1});
    % To discrete from continuous
    model = mc.c2d(G, varargin{2});
  else
    error('No state space model or transfer function')
  end
end
