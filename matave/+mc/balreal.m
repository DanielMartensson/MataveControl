% Generates the balanced realization of a state space model
% Input: sys
% Example 1: model = mc.balreal(sys)
% Author: Daniel Mårtensson, Oktober 2017

function [model] = balreal(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error ('Missing model')
  end
  
  % Get model type
  type = varargin{1}.type;  
  % Check if there is a TF or SS model
  if(strcmp(type, 'SS' ))
    % Get sys
    sys = varargin{1};
    % Get gramians
    Q = mc.gram(sys, 'o');
    P = mc.gram(sys, 'c');
    % Get hankel singular values
    E = diag(mc.hsvd(sys));
    
    % Begin
    Q1 = chol(Q);
    U = chol(eye(size(Q)));
    E1 = chol(E);
    
    T = inv(E1)*U'*Q1;
    
    % Get matrecies
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;
    delay = sys.delay;
    sampleTime = sys.sampleTime;
    
    model = mc.ss(delay, T*A*inv(T), T*B, C*inv(T), D);
    model.sampleTime = sampleTime;
  elseif(strcmp(type, 'TF' ))
    disp('Only state space model are allowed')
  else
    error('This is not TF or SS');
  end
    
end