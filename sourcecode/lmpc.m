% Use linear Model Predictive Control with limits on inputs
% Input: sysd, N(horizon), r(Reference vector), T(End time), u_lb(Min limit input), u_ub(Max limit input), y_lb(Min limit output), y_ub(Max limit output), x0(Initial state)
% Example 1: [y, t, x, u] = lmpc(sysd, N, r, T, u_lb, u_ub, y_lb, y_ub)
% Example 2: [y, t, x, u] = lmpc(sysd, N, r, T, u_lb, u_ub, y_lb, y_ub, x0)
% Author: Daniel Mårtensson
% Update: Added Linear programming too. 2020-04-10

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
    
    % Get the horizon N
    if(length(varargin) >= 2)
      N = varargin{2};
    else
      error('Missing the predict horizon Np');
    end
    
    % Get the reference vector R
    if(length(varargin) >= 3)
      r = varargin{3};
      R = repmat(r, N, 1); % Create the reference vector R = [r1; r2; r1; r2; ... etc]
    else
      error('Missing the reference vector R');
    end
    
    % Get the total time 
    if(length(varargin) >= 4)
      T = varargin{4};
      t = 0:sampleTime:T; % Get time vector - Also return it!
    else
      error('Missing the total time number T');
    end
    
    % Get the lower bound vector u_lb 
    if(length(varargin) >= 5)
      u_lb = varargin{5};
    else
      error('Need lower bound vector u_lb')
    end
    
    % Get the upper bound vector u_ub 
    if(length(varargin) >= 6)
      u_ub = varargin{6};
    else
      error('Need lower bound vector u_ub')
    end
    
    % Get the minimum input signal limit
    if(length(varargin) >= 7)
      y_lb = varargin{7};
    else
      error('Need minimum input signal limit')
    end
    
    % Get the maximum input signal limit
    if(length(varargin) >= 8)
      y_ub = varargin{8};
    else
      error('Need maximum input signal limit')
    end
    
    % Get the initial trajectory vector x
    if(length(varargin) >= 9)
      x = varargin{9};
    else
      x = zeros(size(A, 1), 1);
    end
    
    % Compute the PHI matrix now!
    PHI = phiMat(A, C, N);
    % Compute the GAMMA matrix now
    GAMMA = gammaMat(A, B, C, N);
    % Compute the tuning matrix - Set it to identity matrix
    alpha = 0.001; % Regularization parameter
    Q = alpha*eye(size(GAMMA, 1), size(GAMMA, 1)); 
    % Compute H matrix
    H = GAMMA'*Q*GAMMA;
    % Create initial input signal 
    q = GAMMA'*Q*PHI*x - GAMMA'*Q*R;
    % Choose the lower bounds and upper bounds limits for input
    U_LB = repmat(u_lb, N, 1);
    U_UB = repmat(u_ub, N, 1);
    % Choose the lower bounds and upper bounds limits for output
    A_IN = GAMMA;
    Y_LB = repmat(y_lb, N, 1) - PHI*x;
    Y_UB = repmat(y_ub, N, 1) - PHI*x;
    % Compute the first input signals!
    u = qp([], H, q, [], [], U_LB, U_UB, Y_LB, A_IN, Y_UB);
   
    % This is for LINEAR PROGRAMMING - uncomment them all
    % This is on the form max: c^Tx, St: Ax <= b, x >= 0.
    % Notice that this have Tikhonov Regularization included
    %b = R - PHI*x;
    %CTYPE = repmat(["U"], 1, size(A_IN, 1));
    %VARTYPE = repmat(["C"], 1, size(A_IN, 2));
    %u = glpk((GAMMA'*GAMMA + alpha*eye(size(GAMMA)))'*b, (GAMMA'*GAMMA + alpha*eye(size(GAMMA))), GAMMA'*b, U_LB, [], CTYPE, VARTYPE, -1);

    
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
      u = qp([], H, q, [], [], U_LB, U_UB, Y_LB, A_IN, Y_UB);
      
      % This is for LINEAR PROGRAMMING - uncomment them all
      %b = R - PHI*x;
      %u = glpk((GAMMA'*GAMMA + alpha*eye(size(GAMMA)))'*b, (GAMMA'*GAMMA + alpha*eye(size(GAMMA))), GAMMA'*b, U_LB, [], CTYPE, VARTYPE, -1);
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


function PHI = phiMat(A, C, N)
  
  % Create the special Observabillity matrix
  PHI = [];
  for i = 1:N
    PHI = vertcat(PHI, C*A^i);
  end
  
end

function GAMMA = gammaMat(A, B, C, N)
  
  % Create the lower triangular toeplitz matrix
  GAMMA = [];
  for i = 1:N
    GAMMA = horzcat(GAMMA, vertcat(zeros((i-1)*size(C*A*B, 1), size(C*A*B, 2)),cabMat(A, B, C, N-i+1)));
  end
  
end

function CAB = cabMat(A, B, C, N)
  
  % Create the column for the GAMMA matrix
  CAB = [];
  for i = 0:N-1
    CAB = vertcat(CAB, C*A^i*B);
  end
  
end
