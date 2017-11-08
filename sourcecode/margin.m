% Plot the nyquist diagram of a state space model with margins or a transfer function with margins 
% between frequencies w1 and w2
% Input: sys, G, w1, w2
% Example 1: [Am, phim, wpi, wc] = margin(sys, w1, w2)
% Example 2: [Am, phim, wpi, wc] = margin(G, w1, w2)
% Author: Daniel Mårtensson 2017, Oktober

function [Am, phim, wpi, wc] = margin(varargin)
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
    % Call marin
    margin(G, w1, w2);
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
        
        % Transform the angels so it can show more that -180 degrees
        BodeAngles = angle(H) * 180/pi;
        a_past = BodeAngles(1);
        n = 0;
        NewBodeAngles = [];
        for k = 1:L
          a_now = BodeAngles(k) + n; 
          if(and(a_now > 170 + n, a_past < -170 + n))
            n = n - 180*2;
          else
            a_past = a_now;
          end
          a_now = BodeAngles(k) + n; 
          NewBodeAngles = [NewBodeAngles a_now];
        end
        
        % Get wc, phim
        wc = inf;
        phim = inf;
        flag = false;
        for k = 1:length(H)
          % The dB need to be over 0 for flag = true
          if ((20*log10(abs(H(k))) > 0))
            flag = true;
          end
          % When dB is under 0 and flag = true
          if (and(20*log10(abs(H(k))) <= 0, flag == true ))
            wc = w(k);
            phim = 180 + NewBodeAngles(k);
            break;
          end
        end
        
        % Get wpi, Am
        wpi = inf;
        Am = inf;
        for k = 1:length(H)
          if (NewBodeAngles(k) <= -180)
            wpi = w(k);
            Am = 20*log10(abs(H(k)));
            break;
          end
        end

        figure('Name', sprintf(strcat('Transfer function: ', num2str(i), 'x', num2str(j))), 'NumberTitle', 'off')
        subplot(2,1,1)
        semilogx(w, 20*log10(abs(H)), wc, 0, 'x', linspace(wpi, wpi), linspace(0, Am));
        legend('Mag', 'Wc', 'Am')
        ylabel('Magnitude [dB]');
        title(strcat('Am = ', num2str(abs(Am)), ' dB (at ', num2str(wpi), ' rad/s) , ', 'Phim = ', num2str(abs(phim)), ' deg (at ', num2str(wc), ' rad/s)'))
        grid on
        subplot(2,1,2)
        
        semilogx(w, NewBodeAngles, linspace(wc, wc), linspace(-180, phim - 180), wpi, -180, 'x');
        ylabel('Phase [deg]');
        xlabel('Frequency [rad/s]');
        grid on
        legend('Phase', 'Phim', 'Am')
        
        % Am 
        Am = abs(Am);
             
      end
    end
  else
    error('Only transfer functions and state space models allowed')
  end
end
