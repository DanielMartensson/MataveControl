% Generates the parallel PID controller as a transfer function
% Input: Kp, Ti(Optinal), Td(Optional), Tf(Optinal), Ts(Optinal)
% Example 1: [Gpid] = pid(Kp, Ti, Td, Tf, Ts)
% Author: Daniel Mårtensson, Oktober 2017

function [Gpid] = pid(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error ('Missing parameters')
  end
  
  % Input the parameters
  if(length(varargin) >= 1)
    Kp = varargin{1};
  else
    error('Need to have at least Kp');
  end
  
  % Integrator
  if(length(varargin) >= 2)
    Ti = varargin{2};
  else
    Ti = 0;
  end
  
  % Derivative
  if(length(varargin) >= 3)
    Td = varargin{3};
  else
    Td = 0;
  end
  
  % Low pass filter
  if(length(varargin) >= 4)
    Tf = varargin{4};
  else
    Tf = 0;
  end
  
  % Sampling time
  if(length(varargin) >= 5)
    Ts = varargin{5};
  else
    Ts = 0;
  end
  
  % Build the PID
  Gpid = tf([Kp*Ti*(Td+Tf) Kp*(Ti+Tf) Kp],[Ti*Tf Ti 0]);
  
  % Convert to discrete if needed
  if(Ts > 0)
    Gpid = c2d(Gpid, Ts);
  end
  
end
