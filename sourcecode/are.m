% Solve Algebraic Riccati Equation with the weighting matrices Q and R
% Input: sys, Q, R
% Example 1: [X] = are(sys, Q, R)
% Author: Daniel Mårtensson, Oktober 2017

function [X] = are(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error ('Missing model')
  end
  
  % Get Q R
  if(length(varargin) >= 2)
    Q = varargin{2};
    R = varargin{3};
  else
    error('Missing Q or R');
  end
  
  % Check if Q is symmetric
  if(~issymmetric(Q))
    error('Q needs to be symmetric')
  end
  
  % Get model type
  type = varargin{1}.type;  
  % Check if there is a TF or SS model
  if(strcmp(type, 'SS' ))
    % Get sys
    sys = varargin{1};
    
    % Get sample time
    sampleTime = sys.sampleTime;
    
    % Get matrices
    A = sys.A;
    B = sys.B;
    C = sys.C;
    
    % Get info
    ny = size(C, 1); % Number outputs
    nu = size(B, 2); % Number inputs
    nx = size(A, 1); % Number states
    
    % Check if Q have the right dimension
    if(nx ~= size(Q, 1))
      str = sprintf('Q need to have the dimension %ix%i', nx, nx);
      error(str);
    end
    
    % Check if R have the right dimension
    if(nu ~= size(R, 1))
      str = sprintf('R need to have the dimension %ix%i', nu, nu);
      error(str);
    end
    
    if(sampleTime > 0)
      % Create discrete algebraic riccati equation
      fun = @(X, A, B, Q, R) A'*X*A - X - (A'*X*B)*inv(R + B'*X*B)*(B'*X*A) + Q;
    else
      % Create continuos algebraic riccati equation
      fun = @(X, A, B, Q, R) A'*X + X*A - X*B*inv(R)*B'*X + Q;
    end
    
    % Get initial conditions
    x0 = ones(size(A));
    % Solve algebraic riccati equation
    X = fsolve(@(X) fun(X, A, B, Q, R), x0);
    X = abs(X); % Turn all negative values positive
    
  elseif(strcmp(type, 'TF' ))
    disp('Only state space models only')
  else
    error('This is not TF or SS');
  end
end
