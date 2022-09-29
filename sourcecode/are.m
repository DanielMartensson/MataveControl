% Solve Algebraic Riccati Equation
% Input: sys(State space model for A,B,C,D), Q(Hermitian and same size as A), R(Hermitian and same size as columns of B)
% Input: S(Same size as A), E(Hermitian and same size as A), G(Hermitian and continuous ARE. Same size as rows of B)
% Example 1: [X, K, L] = are(sys, Q, R)
% Example 2: [X, K, L] = are(sys, Q, R, S)
% Example 3: [X, K, L] = are(sys, Q, R, S, E)
% Example 4: [X, K, L] = are(sys, Q, R, S, E, G)
% Author: Daniel Mårtensson, Oktober 2017
% Updated: November 2017
% Updated: July 2020 - Added symmetric semi-positive definitive check for Q and R
% Updated: September 2022 - Added for S, E and G

function [X, K, L] = are(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error ('Missing model')
  end

  % Get Q
  if(length(varargin) >= 2)
    Q = varargin{2};
  else
    error('Missing Q');
  end

  % Get R
  if(length(varargin) >= 3)
    R = varargin{3};
  else
    error('Missing R');
  end

  % Get S
  if(length(varargin) >= 4)
    S = varargin{4};
  else
    % Zero matrix
    sys = varargin{1};
    [nx, nu] = size(sys.B);
    S = zeros(nx, nu);
  end

  % Get E
  if(length(varargin) >= 5)
    E = varargin{5};
  else
    % Identity matrix
    sys = varargin{1};
    E = eye(size(sys.A));
  end

  % Get G
  if(length(varargin) >= 6)
    G = varargin{6};
  else
    % Identity matrix
    sys = varargin{1};
    G = zeros(size(sys.A));
  end


  % Check if Q is hermitian
  if(ishermitian(Q) == 0)
    error('Matrix Q is not hermitian. Try Q = C^T*C');
  end

  % Check if R is hermitian
  if(ishermitian(R) == 0)
    error('Matrix R is not hermitian');
  end

  % Check if E is non-singular
  if(abs(det(E)) <= eps)
    error('Matrix E is singular');
  end

  % Check if G is hermitian
  if(ishermitian(G) == 0)
    error('Matrix G is not hermitian. Try G = -B*B^T');
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
    if(size(Q, 1) == 1)
      R = R*eye(nx); % Scalar to matrix
    elseif(nx ~= size(Q, 1))
      str = sprintf('Q need to have the dimension %ix%i', nx, nx);
      error(str);
    end

    % Check if R have the right dimension
    if(size(R, 1) == 1)
      R = R*eye(nu); % Scalar to matrix
    elseif(and(nu ~= size(R, 1), nu ~= size(R, 2)))
      str = sprintf('R need to have the dimension %ix%i', nu, nu);
      error(str);
    end

    % Check if S have the right dimension
    if(and(nx ~= size(S, 1) , nu ~= size(S, 2)))
      str = sprintf('S need to have the dimension %ix%i', nx, nu);
      error(str);
    end

    % Check if E have the right dimension
    if(nx ~= size(E, 1))
      str = sprintf('E need to have the dimension %ix%i', nx, nx);
      error(str);
    end

    % Check if G have the right dimension
    if(nx ~= size(G, 1))
      str = sprintf('G need to have the dimension %ix%i', nx, nx);
      error(str);
    end

    % Get initial conditions
    x0 = ones(size(A))(:); % Vector

    if(sampleTime > 0)
      % Create discrete algebraic riccati equation and simulate
      [T, X] = ode45(@(t,X)dare(t, X, A, B, Q, R, S, E), [0 100], x0);
    else
      % Create time continuous algebraic riccati equation and simulate
      [T, X] = ode45(@(t,X)care(t, X, A, B, Q, R, S, E, G), [0 100], x0);
    end

    % Get the last value of X and turn it into a matrix_type
    X = reshape(X(size(X, 1), :), size(A)); % Here is the solution!

    % Find K
    if(sampleTime > 0)
      K = inv(B'*X*B + R)*(B'*X*A + S');
    else
      K = inv(R)*(B'*X*E + S');
    end

    % Find L
    if(sampleTime > 0)
      L = eig(A - B*K, E);
    else
      L = eig(A + G*X*E - B*K, E);
    end

  elseif(strcmp(type, 'TF' ))
    disp('Only state space models only')
  else
    error('This is not TF or SS');
  end
end

function [value, isterminal, direction] = care(t, X, A, B, Q, R, S, E, G)

  X = reshape(X, size(A)); % Vector -> Matrix
  value = A'*X*E + E'*X*A + E'*X*G*X*E - (E'*X*B + S)*inv(R)*(B'*X*E + S') + Q; % Value is the derivative of X
  value = value(:); % Matrix -> Vector

  isterminal = 1;
  direction = 0;
end

function [value, isterminal, direction] = dare(t, X, A, B, Q, R, S, E)

  X = reshape(X, size(A)); % Vector -> Matrix
  value = A'*X*A - E'*X*E - (A'*X*B + S)*inv(B'*X*B + R)*(A'*X*B + S)' + Q; % Value is the derivative of X
  value = value(:); % Matrix -> Vector

  isterminal = 1;
  direction = 0;
end
