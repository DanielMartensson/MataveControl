% Generate control law gain matrix L of a state space model by using eigen values vector P
% Input: G, sys, P
% Example 1: [L] = acker(sys, P)
% Example 2: [L] = acker(G, P)
% Author: Daniel Mårtensson, October 2017

function [L] = acker(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error ('Missing input')
  end
  
  % Get model type
  type = varargin{1}.type;  
  % Check if there is a TF or SS model
  if(strcmp(type, 'SS' ))
    % Get model
    sys = varargin{1};
    % Get matrecies
    A = sys.A;
    B = sys.B;
    
    % Check if B is SISO
    if(size(B, 2) > 1)
      error('Only SISO models!')
    end
    
    % Get eigen values
    if(length(varargin) >= 2)
      P = varargin{2};
    else
      error('Missing the closed loop poles')
    end
    
    % Vectorize P
    P = P(:);
    if(size(A,1) ~= length(P))
      error('Poles need to have the same dimension as matrix A')
    end
    
    % Create the control law gain matrix L
    %Formula from Ogata Modern Control Engineering
    L = ctrb(sys)\polyvalm(real(poly(P)), A);
    L = L(size(A,2),:);
    
    % Check if the user has put in very bad pole locations
    P = sort(P);
    nonZeroPoles = find(P ~= 0);
    P = P(nonZeroPoles);
    eigenvalues = sort(eig(A-B*L)); % The state feedback eigenvalues
    eigenvalues = eigenvalues(nonZeroPoles);
    M = abs(P);
    if(max(abs(P-eigenvalues)./M) > .10)
      disp('Warning: Pole locations are in more that 10% error')
    end
    
  elseif(strcmp(type, 'TF' ))
    disp('Only state space models only')
  else
    error('This is not TF or SS');
  end
end
