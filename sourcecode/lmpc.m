% Use linear Model Predictive Control with limits on inputs
% Input: sysd, Np(prediction horizon), Nc(Control horizon), r(Reference vector), T(End time), lb(Min limit input), ub(Max limit input),  x0(Initial state)
% Example 1: [y, t, x, u] = lmpc(sysd, Np, Nc, r, T, lb, ub)
% Author: Daniel Mårtensson

% Can only be still used with Octave due to the QP-command! I will work on that!

function [y, t, X, U] = lmpc(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error ('Missing model')
  endif
  
  % Get model type
  type = varargin{1}.type;  
  % Check if there is a TF or SS model
  if(strcmp(type, 'SS' ))
    % Get A, B, C, D matrecies
    sys = varargin{1};
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;
    delay = sys.delay;
    sampleTime = sys.sampleTime;
    
    % Check if the model is discrete! MPC can only be discrete in this case.
    if sampleTime <= 0
      error('Only discrete state space models')
    end
    
    % Get the predict horizon Np
    if(length(varargin) >= 2)
      Np = varargin{2};
    else
      error('Missing the predict horizon Np');
    end
    
    % Get the control horizon Nc
    if(length(varargin) >= 3)
      Nc = varargin{3};
    else
      error('Missing the control horizon Nc');
    end
    
    % Get the reference vector R
    if(length(varargin) >= 4)
      r = varargin{4};
      R = repmat(r, Np, 1); % Create the reference vector R = [r1; r2; r1; r2; ... etc]
    else
      error('Missing the reference vector R');
    end
    
    % Get the total time 
    if(length(varargin) >= 5)
      T = varargin{5};
      t = 0:sampleTime:T; % Get time vector - Also return it!
    else
      error('Missing the total time number T');
    end
    
    % Get the lower bound vector lb 
    if(length(varargin) >= 6)
      lb = varargin{6};
    else
      lb = []; 
    end
    
    % Get the upper bound vector ub 
    if(length(varargin) >= 7)
      ub = varargin{7};
    else
      ub = [];
    end
    
    % Get the initial trajectory vector x
    if(length(varargin) >= 8)
      x = varargin{8};
    else
      x = zeros(size(A, 1), 1);
    end
    
    % Compute the PHI matrix now!
    PHI = PHImatrix(C, A, Np);
    % Compute the GAMMA matrix now
    GAMMA = GAMMAmatrix(C, A, B, Np, Nc);
    % Compute the tuning matrix - Set it to identity matrix
    Q = eye(size(GAMMA, 1), size(GAMMA, 1));
    % Compute H matrix
    H = GAMMA'*Q*GAMMA;
    % Create initial input signal 
    q = GAMMA'*Q*PHI*x - GAMMA'*Q*R;
    % Choose the lower bounds and upper bounds limits
    LB = repmat(lb, Nc, 1);
    UB = repmat(ub, Nc, 1);
    % Compute the first input signals!
    u = qp([], H, q, [], [], LB, UB);
    
    % Find the optimal input signals U from the QP-formula: J = 0.5*U'H*U + U'*q
    for k = 1:size(t,2) 
      % Return states and input signals
      X(:,k) = x;
      U(:,k) = u(1:size(B, 2)); % First element!
      
      % Compute outputs
      y(:,k) = C*x + D*u(1:size(B, 2)); % size(B, 2) = If multiple inputs...
      
      % Update states and input signals and q matrix
      x = A*x + B*u(1:size(B, 2)); 
      q = GAMMA'*Q*PHI*x - GAMMA'*Q*R;
      u = qp([], H, q, [], [], LB, UB);
    end
    
    % Change t and y vector and u so the plot look like it is discrete - Important!
    for(i = 1:2:length(y)*2)
      leftPart = y(:,1:i);
      rightPart = y(:,(i+1):end);
      y = [leftPart y(:,i) rightPart];
    end
        
    for(i = 1:2:length(t)*2)
      leftPart = t(1:i);
      rightPart = t((i+1):end);
      t = [leftPart t(i) rightPart];
    end
    
    for(i = 1:2:length(U)*2)
      leftPart = U(:,1:i);
      rightPart = U(:,(i+1):end);
      U = [leftPart U(:,i) rightPart];
    end
    
    for(i = 1:2:length(X)*2)
      leftPart = X(:,1:i);
      rightPart = X(:,(i+1):end);
      X = [leftPart X(:,i) rightPart];
    end
        
    % Just remove the first one 
    t = t(:,2:length(t));
    % And the last one
    y = y(:,1:(length(y)-1));
    % And for U and X too
    U = U(:,1:(length(U)-1));
    X = X(:,1:(length(X)-1));
    % Now we have two vectors which look like a discrete signal
    
    % Plot - How many subplots?
    for i = 1:size(C,1)
      subplot(size(C,1),1,i)
      plot(t, y(i,:)); 
      ylabel(strcat('y', num2str(i)));
      if (sampleTime > 0)
        xlabel(strcat(num2str(sampleTime), ' time unit/sample'));
      else
        xlabel('Time units');
      end
      grid on
    end
    
  endif
endfunction

function [PHI] = PHImatrix(C, A, Np)
  
  PHI = [];
  for i = 1:(Np)
    PHI = [PHI; C*A^i];
  endfor
  
endfunction

function [GAMMA] = GAMMAmatrix(C, A, B, Np, Nc)
  PHI = [];
  GAMMA = [];
  
  for j = 1:Nc
    for i = (1-j):(Np-j)
      
      if i < 0
        PHI = [PHI; 0*C*A^i*B];
      else
        PHI = [PHI; C*A^i*B];
      endif
      
    endfor
    
    % Add to PHI
    GAMMA = [GAMMA PHI];
    % Clear F
    PHI = [];
  endfor
  
endfunction
