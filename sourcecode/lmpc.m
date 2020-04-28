% Use linear Model Predictive Control
% Input: sysd(Discrete state space model), N(Horizon), t(Time vector), regularization(optimal), x0(Initial state, optimal)
% Output: y(Output signal), T(Discrete time vector), X(State vector), U(Output signal)
% Example 1: [y, T, X, U] = lmpc(sysd, R, t)
% Example 2: [y, T, X, U] = lmpc(sysd, R, t, regularization)
% Example 3: [y, T, X, U] = lmpc(sysd, R, t, regularization, x0)
% Author: Daniel Mårtensson
% Update: Replaced QP solver with LP solver due to CControl library - Sorry! 

function [y, T, X, U] = lmpc(varargin)
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
    % Check if the model is discrete! MPC can only be discrete in this case.
    if sys.sampleTime <= 0
      error('Only discrete state space models');
    end
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;
    delay = sys.delay;
    sampleTime = sys.sampleTime;
    
    % Get the reference vector R
    if(length(varargin) >= 2)
      R = varargin{2};
      if(size(R, 1) ~= size(B, 2))
        error("Use the same dimension of R as nu(number of inputs)");
      end
    else
      error('Missing the reference vector R');
    end
    
    % Get the total time 
    if(length(varargin) >= 3)
      t = varargin{3};
      if(length(t) ~= length(R))
        error("t and R need to have the same length");
      end
    else
      error('Missing the time vector');
    end
    
    % Get the regularization parameter
    if(length(varargin) >= 4)
      regularization = varargin{4};
    else
      regularization = 0;
    end
    
    % Get the initial trajectory vector x
    if(length(varargin) >= 5)
      x = varargin{5};
    else
      x = zeros(size(A, 1), 1);
    end
    
    %% Solve this formula: R = PHI*x + GAMMA*U, where we want U
    
    % Compute the PHI and GAMMA matrix now with horizon length of R
    PHI = phiMat(A, C, length(R));
    GAMMA = gammaMat(A, B, C, length(R));
    
    % We using Tikhonov regularization when we are using linear programming: Max c^T, S.t: Ax <= b, x >= 0
    % clp = (A'*A + lambda*I)'*b
    % blp = A'*b
    % alp = A'*A + lambda*I
    alp = GAMMA'*GAMMA + regularization*eye(size(GAMMA'*GAMMA));
    iteration_limit = 200;
    u = zeros(size(alp, 2), size(B, 2)); % Prevent over shoot from the beginning
    
    % Find the optimal input signals U from the QP-formula: J = 0.5*U'H*U + U'*q
    for k = 1:size(t,2) 
      % Return states and input signals
      X(:,k) = x;
      U(:,k) = u(1:size(B, 2)); % First element!
      
      % Compute outputs
      y(:,k) = C*x + D*u(1:size(B, 2)); % size(B, 2) = If multiple inputs...
      
      % Update states
      x = A*x + B*u(1:size(B, 2)); 
      
      % Update the constraints and objective function
      clp = (GAMMA'*GAMMA + regularization*eye(size(GAMMA'*GAMMA)))'*(R(:, k) - PHI*x);
      blp = GAMMA'*(R(:, k) - PHI*x);
      u = linprog2(clp, alp, blp, 0, iteration_limit);
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
    T = t(:,2:length(t));
    % And the last one
    y = y(:,1:(length(y)-1));
    % And for U and X too
    U = U(:,1:(length(U)-1));
    X = X(:,1:(length(X)-1));
    % Now we have two vectors which look like a discrete signal
    
    % Plot - How many subplots?
    for i = 1:size(C,1)
      subplot(size(C,1),1,i)
      plot(T, y(i,:)); 
      ylabel(strcat('y', num2str(i)));
      if (sampleTime > 0)
        xlabel(strcat(num2str(sampleTime), ' time unit/sample'));
      else
        xlabel('Time units');
      end
      grid on
    end
    
  end
end


% This simplex method has been written as it was C code
function [x] = linprog2(c, A, b, max_or_min, iteration_limit)
  row_a = size(A, 1);
  column_a = size(A, 2);
  
  if(max_or_min == 0)
    % Maximization - Regular simplex method
    x = opti(c, A, b, row_a, column_a, max_or_min, iteration_limit);
  else
    % Minimization - The dual simplex method
    x = opti(b, A', c, column_a, row_a, max_or_min, iteration_limit);
  end
end


function [x] = opti(c, A, b, row_a, column_a, max_or_min, iteration_limit)
  
  % Clear the solution
  if(max_or_min == 0)
    x = zeros(column_a, 1);
  else
    x = zeros(row_a, 1);
  end
  
  % Create the tableau
  tableau = zeros(row_a + 1, column_a + row_a + 2);
  j = 1;
  for i = 1:row_a
    % First row
    tableau(i, 1:column_a) = A(i, 1:column_a);
    
    % Slack variable s
    j = column_a + i;
    tableau(i, j) = 1;
    
    % Add b vector
    tableau(i, column_a + row_a + 2) = b(i);
  end
  
  % Negative objective function
  tableau(row_a + 1, 1:column_a) = -c(1:column_a);
 
  % Slack variable for objective function
  tableau(row_a + 1, column_a + row_a + 1) = 1;
  
  % Do row operations
  entry = -1.0; % Need to start with a negative number because MATLAB don't have do-while! ;(
  pivotColumIndex = 0;
  pivotRowIndex = 0;
  pivot = 0.0;
  value1 = 0.0;
  value2 = 0.0;
  value3 = 0.0;
  smallest = 0.0;
  count = 0;
  while(entry < 0) % Continue if we have still negative entries
    % Find our pivot column
    pivotColumIndex = 1;
    entry = 0.0;
    for i = 1:column_a + row_a + 2
      value1 = tableau(row_a + 1, i);
      if(value1 < entry)
        entry = value1;
        pivotColumIndex = i;
      end
    end
    
    % If the smallest entry is equal to 0 or larger than 0, break
    if(or(entry >= 0.0, count >= iteration_limit))
      break;
    end 
    
    % Find our pivot row
    pivotRowIndex = 1;
    value1 = tableau(1, pivotColumIndex); % Value in pivot column
    value2 = tableau(1, column_a+row_a+2); % Value in the b vector
    smallest = value2/value1; % Initial smalles value1
    for i = 2:row_a
      value1 = tableau(i, pivotColumIndex); % Value in pivot column
      value2 = tableau(i, column_a+row_a+2); % Value in the b vector
      value3 = value2/value1;
      if(or(and(value3 > 0, value3 < smallest), smallest < 0))
        smallest = value3;
        pivotRowIndex = i;
      end 
    end
    
    % We know where our pivot is. Turn the pivot into 1
    % 1/pivot * PIVOT_ROW -> PIVOT_ROW
    pivot = tableau(pivotRowIndex, pivotColumIndex); % Our pivot value
    for i = 1:column_a + row_a + 2
      value1 = tableau(pivotRowIndex, i); % Our row value at pivot row
      tableau(pivotRowIndex, i) = value1 * 1/pivot; % When value1 = pivot, then pivot will be 1
    end
    
    % Turn all other values in pivot column into 0. Jump over pivot row
		% -value1* PIVOT_ROW + ROW -> ROW
    for i = 1:row_a + 1
      if(i ~= pivotRowIndex)
        value1 = tableau(i, pivotColumIndex); %  This is at pivot column
        for j = 1:column_a+row_a+2
          value2 = tableau(pivotRowIndex, j); % This is at pivot row
          value3 = tableau(i, j); % This is at the row we want to be 0 at pivot column
          tableau(i, j) = -value1*value2 + value3;
        end
      end
    end
    
    % Count for the iteration
		count = count + 1;
  end
  
  % If max_or_min == 0 -> Maximization problem
  if(max_or_min == 0)
    % Now when we have shaped our tableau. Let's find the optimal solution. Sum the columns
    for i = 1:column_a
      value1 = 0; % Reset
      for j = 1:row_a + 1
        value1 = value1 + tableau(j, i); % Summary
        value2 = tableau(j, i); %  If this is 1 then we are on the selected
        
        % Check if we have a value that are very close to 1
        if(and(and(value1 < 1 + eps, value1 > 1 - eps), value2 > 1 - eps))
          x(i) = tableau(j, column_a+row_a+2);
        end
      end
    end
  else
    % Minimization (The Dual method) - Only take the bottom rows on the slack variables
    for i = 1:row_a
      x(i) = tableau(row_a+1, i + column_a); % We take only the bottom row at start index column_a
    end
  end
    
end


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
