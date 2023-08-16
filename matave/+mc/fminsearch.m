% Nonlinear programming solver. Searches for the minimum of a problem specified by
% f(x)
% min
% Input: fun(objective function), x0(initial state), max_iterations, tolerance
% Output: x(solution), fval(fun(x)), flag(status), iterations
% Example 1: [x, fval, flag, iterations] = mc.fminsearch(fun, x0)
% Example 2: [x, fval, flag, iterations] = mc.fminsearch(fun, x0, max_iterations)
% Example 1: [x, fval, flag, iterations] = mc.fminsearch(fun, x0, max_iterations, tolerance)
% Author: Daniel Mårtensson, Augusti 2023

function [x, fval, flag, iterations] = fminsearch(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error('Missing inputs')
  end

  % Get objective function
  if(length(varargin) >= 1)
    fun = varargin{1};
  else
    error('Missing objective function')
  end

  % Get the initial state
  if(length(varargin) >= 2)
    x0 = varargin{2};
  else
    error('Missing initial state');
  end

  % Get the maximum iterations
  if(length(varargin) >= 3)
    max_iterations = varargin{3};
  else
    max_iterations = 10000;
  end

  % Get the epsilon
  if(length(varargin) >= 4)
    tolerance = varargin{4};
  else
    tolerance = 1.192092896e-07; % FLT_EPSILON in C
  end

  % Initialize  parameters
  lambda = 0.7;
  rho = 1;
  chi = 2;
  gama = 0.5;
  sigma = 0.5;

  % Get the dimension of the initial state
  n = length(x0);

  % Create holder for solution
  p = zeros(n, n + 1);

  % If we made x0 into an array, but we want it into a vector instead
  p(:, 1) = x0(:);

  % Compulate the starting point
  e = zeros(n, 1);
  for i = 2:(n + 1)
      e(i-1) = 1;
      p(:, i) = p(:, 1) + lambda*e;
      e(i-1) = 0;
  end

  % Compulate the value
  value = zeros(1, n+1);
  for i = 1:(n + 1)
      value(i) = fun(p(:, i));
  end

  % Sorting and ready to iterate
  [value, indexes] = sort(value);
  p = p(:, indexes);

  % Start iteration
  iterations = 0;
  while iterations < max_iterations
      % Breaking condition
      mean = (value(1) + value(n) + value(n + 1))/3;
      indicator = sqrt(((value(1)-mean)^2+(value(n) - mean)^2 + (value(n + 1)-mean)^2)/3);
      if indicator < tolerance
          break
      end

      % Selet the three point
      best_point = p(:, 1);
      best_point_value = value(1);
      sub_worst_point = p(:, n);
      sub_worst_point_value = value(n);
      worst_point = p(:, n + 1);
      worst_point_value = value(n + 1);

      % Reflection
      center = (best_point + sub_worst_point)/2;
      reflection_point = center + rho*(center - worst_point);
      reflection_point_value = fun(reflection_point);

      if reflection_point_value < best_point_value
          % Expansion
          expansion_point = center + chi*(reflection_point - center);
          expansion_point_value = fun(expansion_point);
          if expansion_point_value < reflection_point_value
              p(:, n + 1) = expansion_point;
              value(n + 1) = expansion_point_value;
          else
              p(:, n + 1) = reflection_point;
              value(n + 1) = reflection_point_value;
          end
      else
          if reflection_point_value < sub_worst_point_value
              p(:, n + 1) = reflection_point;
              value(n + 1) = reflection_point_value;
          else
              % Outside Constraction
              shrink = false;
              if reflection_point_value < worst_point_value
                  outside_constraction_point = center + gama*(reflection_point - center);
                  outside_constraction_point_value = fun(outside_constraction_point);
                  if outside_constraction_point_value < worst_point_value
                      p(:, n + 1) = outside_constraction_point;
                      value(n + 1) = outside_constraction_point_value;
                  else
                      shrink = true;
                  end
                  % Inside Constraction
              else
                  inside_constraction_point = center + gama*(worst_point - center);
                  inside_constraction_point_value = fun(inside_constraction_point);
                  if inside_constraction_point_value < worst_point_value
                      p(:, n + 1) = inside_constraction_point;
                      value(n + 1) = inside_constraction_point_value;
                  else
                      shrink = true;
                  end
              end

              % Shrinkage
              if shrink
                  for i = 2:n+1
                      p(:, i) = best_point + sigma*(p(:, i) - best_point);
                      value(i) = fun(p(:, i));
                  end
              end
          end
      end

      % Resort and ready to iterate
      [value, indexes] = sort(value);
      p = p(:, indexes);
      iterations = iterations + 1;
  end


  % Solution and result
  x = p(:,1);
  fval = value(:, 1);
  if iterations < max_iterations
      flag = true;
  else
      flag = false;
  end
end
