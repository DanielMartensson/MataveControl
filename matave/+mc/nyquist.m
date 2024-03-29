% Plot the nyquist diagram of a state space model or a transfer function 
% between given frequencies w1 and w2
% Input: sys, G
% Example 1:  mc.nyquist(sys, w1, w2)
% Example 2:  mc.nyquist(G, w1, w2)
% Author: Daniel Mårtensson, 2017 Oktober

function [reval] = nyquist(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error('Missing model')
  end
  
  % Check if there is any input
  if(length(varargin) < 3)
    w1 = 0.01;
    w2 = 100;
  else
    w1 = varargin{2};
    w2 = varargin{3};
  end
  
   % Get the type
  type = varargin{1}.type;
  % Check if there is a TF or SS model
  if(strcmp(type, 'SS' ))
    % SS to TF
    G = mc.ss2tf(varargin{1});
    % Call nyquist
    mc.nyquist(G, w1, w2);
  elseif(strcmp(type, 'TF' ))
    % If there is a MIMO TF
    G = varargin{1};
    for i = 1:size(G,1)
      for j = 1:size(G,2)
        % Get numerator vector and denomerator vector
        a = G(i,j).num;
        b = G(i,j).den;
        % Get delay
        delay = G(i,j).delay;
        % Get sample time
        sampleTime = G(i,j).sampleTime;
        
        % Numerator and denomerator need to be the same length
        if(length(a) > length(b))
          b = [zeros(1, size(a,2) - size(b,2)) b];
        elseif(length(a) < length(b))
          a = [zeros(1, size(b,2) - size(a,2)) a];
        end
        
        L = 10000;                                 % Number of frequency elements - Need to be 10000 for the nyquist plot
        N = length(b);                             % Number of denomerators                
        w = logspace(log10(w1), log10(w2), L);     % Angular frequencies
        % Evaluate transfer function
        H = zeros(1, L);
        h = sampleTime; 
        if(sampleTime > 0) % Discrete model
          for k = 1 : L
            H(k) = (a*fliplr((exp(1i*w(k)*h)).^(0 : N-1)).')/(b*fliplr((exp(1i*w(k)*h)).^(0 : N-1)).')*exp(-delay*exp(1i*w(k)*h));
          end
        else
          for k = 1 : L
            H(k) = (a*fliplr((1i*w(k)).^(0 : N-1)).')/(b*fliplr((1i*w(k)).^(0 : N-1)).')*exp(-delay*1i*w(k));
          end
        end
        % Done!
        % Plot nyquist diagram

        figure('Name', sprintf(strcat('Transfer function: ', num2str(i), 'x', num2str(j))))
        plot([real(H) nan real(H)], [imag(H) nan -imag(H)],-1, 0 ,'+')
        title('Nyquist diagram')
        xlabel('Real axis')
        ylabel('Imaginary axis')
        grid on
      end
    end
  else
    error('Only transfer functions and state space models allowed')
  end
end
