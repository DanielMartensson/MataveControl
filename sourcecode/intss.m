% Add internal integration into a existing state space model
% Input: sys
% Example 1: [sys] = intss(sys)
% Author: Daniel Mårtensson, Februari 2018

function [model] = intss(varargin)
  if(isempty(varargin{1}))
    error ('Missing model')
  end
  
  % State space
  if(strcmp(varargin{1}.type, 'SS' ))
    % Get SS model
    sys = varargin{1}; 
    Am = sys.A;
    Bm = sys.B;
    Cm = sys.C;
    % No D-matrix here!
    
    % Get the size of Cm
    [q, n] = size(Cm);
    
    % Create the augmented state space model! 
    % Litterature: Liuping Wang(2009)
    
    % But if the model is discrete?
    if sys.sampleTime > 0
      Im = eye(q, q);
      I = eye(q, q);
      B = [Bm; Cm*Bm];
    else
      Im = zeros(q, q);
      I = eye(q, q);
      m = size(Bm, 2);
      B = [Bm; zeros(q,m)];
    end
    % Create the rest of the matrecies
    Om = zeros(q, n);
    A = [Am Om'; Cm*Am Im];
    C = [Om I];
    
    % Get delay
    delay = sys.delay;
    
    % Create model
    model = ss(delay, A, B, C); % D matrix automatic 
    
    % Get sample time
    model.sampleTime = sys.sampleTime;
    
  else
    error('No transfer function')
  end
end
