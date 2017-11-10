% Generates the solution to the sylvester equation
% Input: sys
% Example [X] = sylvester(A, B, C)
% Author: Daniel Mårtensson, November 2017

function [X] = sylvester(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error ('Missing A')
  else
    A = varargin{1};
  end
  
  % Check if missing B
  if(length(varargin) >= 2)
    B = varargin{2};
  else
    error('Missing B');
  end
  
  % Check if missing C
  if(length(varargin) >= 3)
    C = varargin{3};
  else
    error('Missing C');
  end
  
  
  % Check if B is square
  if(size(B, 1) ~= size(B,2))
    error('B is not square')
  end
  
  % Check if C has the same rows as A
  if(size(C, 1) ~= size(A, 1))
    error('C need to have the same rows as A');
  end
  
  % Check if C has the same columns as B
  if(size(C, 2) ~= size(B, 2))
    error('C need to have the same columns as B');
  end
  
  % Compute with kronecker
  X = (kron(eye(size(A)), A) + kron(B', eye(size(B))))\C(:);
  X = reshape(X, size(A)); % Reshape before you send the solution back
end