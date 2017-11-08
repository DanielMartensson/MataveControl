% Plot the nyquist diagram of a state space model or a transfer function to find the 
% 3 dB gain drop between frequencies w1 and w2
% Input: G, sys, w1, w2
% Example 1: [drop] = dbdrop(sys, w1, w2)
% Example 2: [drop] = dbdrop(G, w1, w2)
% Author: Daniel Mårtensson, Oktober 2017

function [drop] = dbdrop(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error('Missing model')
  end
  
  % Check if there is any input
  if(length(varargin) < 3)
    error('Missing frequencies')
  end
  
  w1 = varargin{2};
  w2 = varargin{3};

  
   % Get the type
  type = varargin{1}.type;
  % Check if there is a TF or SS model
  if(strcmp(type, 'SS' ))
    % SS to TF
    G = ss2tf(varargin{1});
    % Call dBdrop
    dBdrop(G, w1, w2);
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
        
        L = 1000;                                  % Number of frequency elements
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
        % Plot bode diagram
        
        dBinitial = 20*log10(abs(H(1))) - 3;
        dropWc = NaN;
        for k = 1 : L
          if(20*log10(abs(H(k))) <= dBinitial)  
            drop = w(k);
            break;
          end
        end
        

        figure('Name', sprintf(strcat('Transfer function: ', num2str(i), 'x', num2str(j))), 'NumberTItle', 'off')
        semilogx(w, 20*log10(abs(H)), [drop drop], [dBinitial (dBinitial + 3)]);
        ylabel('Magnitude [dB]');
        grid on
        legend(strcat('3 dB drop: ', num2str(drop), ' rad/s')) 
             
      end
    end
  else
    error('Only transfer functions and state space models allowed')
  end
end
