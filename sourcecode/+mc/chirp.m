
% Chirp - Generates a frequency signal
% Input: t(time), A(amplitude, optional)
% Output: u(signal), fs(sampling frequency)
% Example 1: [u, fs] = chirp(t);
% Example 1: [u, fs] = chirp(t, A);
% Author: Daniel Mårtensson, Oktober 29:e 2022

function [u, fs] = chirp(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error('Missing inputs')
  end

  % Get time
  if(length(varargin) >= 1)
    t = varargin{1};
  else
    error('Missing time');
  end

  % Get amplitude
  if(length(varargin) >= 2)
    A = varargin{2};
  else
    A = 1;
  end

  % Create the length
  N = length(t);

  % Create the difference chirp duration
  T = t(end) - t(1);

  % Create the sampling frequency
  fs = 1/(T/N);

  % Create the start frequency
  f0 = 0;

  % Create the final frequency
  f1 = fs/2;

  % Create the chirp rate
  c = (f1-f0)/T;

  % Create the frequency signal
  u = A*sin(2*pi*(c/2*t.^2 + f0*t));

end
