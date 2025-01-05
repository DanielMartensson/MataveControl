% This is quadratic programming with Hildreth's method
% Min 1/2x^TQx + c^Tx
% S.t Ax <= b
%
% If you want to do maximization, then turn Q and c negative. The constraints are the same
%
% Max 1/2x^T(-Q)x + (-c)^Tx
% S.t Ax <= b
%
% Input: Q(Symmetric matrix), c(Objective function), A(Constraint matrix), b(Constraint vector)
% Output: x(Solution vector), solution(boolean flag)
% Example 1: [x, solution] = mc.quadprog(Q, c, A, b)
% Author: Daniel Mårtensson 2022 September 3

function [x, solution] = quadprog(Q, c, A, b)
  % Assume that the solution is true
  solution = true;

  % Same as in C code for Functions.h at CControl
  MIN_VALUE = 1e-11;
  MAX_ITERATIONS = 10000;

  % Unconstrained solution
  x = -linsolve(Q, c);

  % Constraints difference
  K = b - A*x;

  % Check constraint violation
  if(sum(K <= 0) >= 0)
    return; % No violation
  end

  % Create P
  P = linsolve(Q, A');

  % Create H = A*Q*A'
  H = A*P;

  % Solve lambda from H*lambda = -K, where lambda >= 0
  [m, n] = size(K);
  lambda = zeros(m, n);
  for km = 1:MAX_ITERATIONS
    lambda_p = lambda;

    % Use Gauss Seidel
    for i = 1:m
      w = -1.0/H(i,i)*(K(i) + H(i,:)*lambda - H(i,i)*lambda(i));
      lambda(i) = max(0, w);
    end

    % Check if the minimum convergence has been reached
    w = (lambda - lambda_p)'*(lambda - lambda_p);
    if (w < MIN_VALUE)
      break;
    end

    % Check if the maximum iteration have been reached
    if(km == MAX_ITERATIONS)
      solution = false;
      return;
    end
  end

  % Find the solution: x = -inv(Q)*c - inv(Q)*A'*lambda
  x = x - P*lambda;
end
