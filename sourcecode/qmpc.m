% Use Model Predictive Control with integral action and quadratic programming 
% Input: sysd(Discrete state space model), N(Horizon number), R(Reference vector), T(End time), a(lambda regularization parameter), I(integral parameter 0 to 1), x0(Initial state, optimal)
% Output: y(Output signal), T(Discrete time vector), X(State vector), U(Output signal)
% Example 1: [Y, T, X, U] = qmpc(sysd, N, R, T)
% Example 2: [Y, T, X, U] = qmpc(sysd, N, R, T, a)
% Example 3: [Y, T, X, U] = qmpc(sysd, N, R, T, a, I)
% Example 4: [Y, T, X, U] = qmpc(sysd, N, R, T, a, I, x0)
% Author: Daniel Mårtensson 2022 September 3

function [Y, T, X, U] = qmpc(varargin)
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

    % Get the horizon number
    if(length(varargin) >= 2)
      N = varargin{2};
    else
      error('Missing the horizon number');
    end

    % Get the reference vector R
    if(length(varargin) >= 3)
      try
        R = repmat(varargin{3}, N/length(varargin{3}), 1);
      catch
        error('Try to change the horizon N number so it can be divided with length of reference R vector');
      end
    else
      error('Missing the reference vector R');
    end

    % Get the total time
    if(length(varargin) >= 4)
      t = 0:sampleTime:varargin{4};
    else
      error('Missing the end time T');
    end

    % Get the lambda
    if(length(varargin) >= 5)
      a = varargin{5};
    else
      a = 0;
    end

    % Get the integral action parameter
    if(length(varargin) >= 6)
      I = varargin{6};
    else
      I = 0; % Max integral action
    end

    % Get the initial trajectory vector x
    if(length(varargin) >= 7)
      x = varargin{7};
    else
      x = zeros(size(A, 1), 1);
    end

    % Check if the system has integration behaviour already
    abseigenvalues = abs(pole(sys));
    if(max(abseigenvalues) < 1)
      % Add integral action - It's very good and pratical!
      % A = [A B; 0 I]
      % B = [0; I]
      % C = [C 0]
      % x = [x; u(k-1)]
      am = size(A, 1);
      bn = size(B, 2);
      cm = size(C, 1);
      A = [A B; zeros(bn, am) eye(bn, bn)];
      B = [zeros(am, bn); eye(bn, bn)];
      C = [C zeros(cm, bn)];
      x = [x; zeros(bn, 1)];
    end

    % Check if it's MATLAB or Octave
    isOctave = exist('OCTAVE_VERSION', 'builtin') ~= 0;

    % Find the observability matrix PHI and lower triangular toeplitz matrix GAMMA of C*PHI
    PHI = phiMat(A, C, N);
    GAMMA = gammaMat(A, B, C, N);

    % Solve: R = PHI*x + GAMMA*U with quadratic programming: Min: 1/2x^TQx + c^Tx, S.t: Ax <= b, x >= 0
    % cqp = (GAMMA'*GAMMA + lambda)'*(R - PHI*x)
    % bqp = GAMMA'*(R - PHI*x)
    % aqp = GAMMA'*GAMMA + lambda
    % qqp = GAMMA'*GAMMA + lambda
    lambda = a*eye(size(GAMMA));
    qqp = GAMMA'*GAMMA + lambda;
    aqp = GAMMA;

    % Loop and save the data
    past_inputs = zeros(1, size(B, 2));
    delta = zeros(1, size(B, 2));
    for k = 1:length(t)
      % Return states and input signals
      X(:,k) = x;
      U(:,k) = delta; % First element!

      % Compute outputs
      Y(:,k) = C*x + D*delta; % size(B, 2) = If multiple inputs...

      % Update states
      % For applying this MPC regulator in reality, then state vector x should be the "sensors" if C = identity and D = 0
      x = A*x + B*delta;

      % Update the constraints and objective function
      cqp = GAMMA'*PHI*x - GAMMA'*R;
      bqp = R-PHI*x;

      % Quadratic programming
      if(isOctave == 1)
        [u, ~, e] = qp ([], qqp, cqp, [], [], [], [], [], aqp, bqp);
        if(e.info == 3)
          error('Quadratic programming QP could not optimize input signals. Try increase the horizion N number.');
        end
      else
        [u, solution] = quadprog2(qqp, cqp, aqp, bqp); % Used for MATLAB users
        if(solution == false)
          error('Quadratic programming quadprog could not optimize input signals. Try to decrease the horizion N number or remove/change lambda regularization.');
        end
      end

      % Do integral action by compare new inputs with old inputs
      delta = u(1:size(B, 2)) - I*past_inputs;
      past_inputs = u(1:size(B, 2));
    end

    % Change t and y vector and u so the plot look like it is discrete - Important!
    for(i = 1:2:length(Y)*2)
      leftPart = Y(:,1:i);
      rightPart = Y(:,(i+1):end);
      Y = [leftPart Y(:,i) rightPart];
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
    Y = Y(:,1:(length(Y)-1));
    % And for U and X too
    U = U(:,1:(length(U)-1));
    X = X(:,1:(length(X)-1));
    % Now we have two vectors which look like a discrete signal

    % Plot - How many subplots?
    for i = 1:size(C,1)
      subplot(size(C,1),1,i)
      plot(T, Y(i,:));
      ylabel(strcat('y', num2str(i)));
      if (sampleTime > 0)
        xlabel(strcat(num2str(sampleTime), ' time unit/sample'));
      else
        xlabel('Time units');
      end
      grid on
    end
  end
  title('Model Predictive Control With Integral Action And Quadratic Programming')
end

function [x, solution] = quadprog2(Q, c, A, b)
  x = -linsolve(Q, c);
  solution = true;

  [n1,m1]=size(A);
  k = 0;
  for i=1:n1
    if (A(i,:)*x > b(i))
      k = k +1;
    end
  end
  if (k==0)
    return;
  end

  X = linsolve(Q, A');
  P = A*X;

  d = A*x;
  d = -d + b;

  [n,m] = size(d);
  lambda = zeros(n,m);
  for km = 1:255
   lambda_p = lambda;
   for i=1:n
    w = P(i,:)*lambda - P(i,i)*lambda(i,1) + d(i, 1);
    lambda(i,1) = max(0,-w/P(i,i));
   end
   w = (lambda - lambda_p)'*(lambda - lambda_p);
   if (w < 10e-7)
       break;
   end
   if(km == 255)
      solution = false;
      return;
    end
  end

  x = x - X*lambda;
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
