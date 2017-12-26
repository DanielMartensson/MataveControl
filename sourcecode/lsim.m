% Do a linear simulation of a transfer function or a state space model
% Input: G, sys, u, t, x0(optional)
% Example 1: [y] = lsim(G,u,t)
% Example 2: [y] = lsim(sys,u,t)
% Example 3: [y] = lsim(sys,u,t,x0)
% Author: Daniel Mårtensson, September 2017


function [y] = lsim(varargin)
  % Check if there is some input arguments or it's not a model
  if(isempty(varargin{1}))
    error ('Missing model')
  end

  if(strcmp(varargin{1}.type,'SS'))
    % Get A, B, C, D and delay
    model = varargin{1};
    A = model.A;
    B = model.B;
    C = model.C; % Will not be used
    D = model.D; % Will not be used
    delay = model.delay;
    sampleTime = model.sampleTime;
    
    % Get input
    u = varargin{2}; % In signal vector
    % Check in signal vector
    if(size(u, 1) ~= size(B, 2))
      error('In signal vecor has not the same rows as B matrix')
    end
    
    % Get time
    t = varargin{3}; 
    % Check t vector
    if(size(t, 1) > 1)
      error('No double time vector are allowed')
    end
    
    % Check if t and u have the same length
    lengthTime = length(t);
    lengthSignal = length(u);
    if(lengthTime ~= lengthSignal)
      error('Input and time has not the same length')
    end
    
    % Get the initial state vector
    if(length(varargin) >= 4)
      x = varargin{4}(:); % Initial state vector
      if(size(x, 1) ~= size(A, 1))
        error('Initial state vector vecor has not the same rows as A matrix')
      end
    else
      % Does not exist - create one then!
      x = zeros(size(A, 1), 1); % Assume x0 = [0; 0; 0; ..... ; 0]
    end
    
    
    % Create discrete model - Important for simulation
    if(sampleTime > 0) % Allready Discrete!
      sysd = varargin{1};
    else % Time continous
      sysd = c2d(varargin{1}, t(2) - t(1)); % Sample time is the difference t(2) - t(1)
    end
    
    % Discrete matrecies
    Ad = sysd.A;
    Bd = sysd.B;
    Cd = sysd.C;
    Dd = sysd.D;
    
    
    % Create the delayed input. 
    % If t(1,1) = delay = no delay    
    %for i = 1:size(t,2)
      %if(t(1,i) >= delay)
        %delayedInput = zeros(size(u,1), i); % The same row widt as signal u
        %break;
      %end
    %end
    
    % Simulation 
    for k = 1:size(t,2) 
      %delayedInput = [u(:,k) delayedInput(:, 1:(length(delayedInput) - 1))];
      %y(:,k) = C*x + D*delayedInput(:,end);
      %x = Ad*x + Bd*delayedInput(:,end); % Update state vector
      
      y(:,k) = Cd*x + Dd*u(:,k);
      x = Ad*x + Bd*u(:,k); % Update state vector
    end

    
    % If we have a sample time bigger that 0, then we need to make sure
    % that the signal look like it's in discrete form.
    % If the model has the sample time t(2) - t(1)
    % the model will be inteprented as time continous
    if(sampleTime > 0)
      % Change t and y vector so the plot look like it is discrete
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
        
      % Just remove the first one 
      t = t(:,2:length(t));
      % And the last one
      y = y(:,1:(length(y)-1));
      % Now we have two vectors which look like a discrete signal
    end
    
    % Plot - How many subplots?
    for i = 1:size(C,1)
      subplot(size(C,1),1,i)
      plot(t, y(i,:)); 
      ylabel(strcat('y', num2str(i)));
      xlabel('Timeunits');
      grid on
    end
    
  elseif(strcmp(varargin{1}.type,'TF'))
    % TF to SS
    sys = tf2ss(varargin{1}, 'OCF');
    
    % Get input
    u = varargin{2}; % In signal vector
    t = varargin{3}; % Time vector
    
    % Check the initial state vector
    if(length(varargin) >= 4)
      x0 = varargin{4}(:); % Initial state vector
      if(size(x0, 1) ~= size(sys.A, 1))
        error('Initial state vector vecor has not the same rows as A matrix')
      end
    else
      x0 = zeros(size(sys.A, 1), 1); % Assume x0 = 0
    end
    
    % Check simulation method
    %if(length(varargin) >= 5)
      %method = varargin{5};
    %else
      %method = 'lsim';
    %end
    
    % Call lsim
    y = lsim(sys, u, t, x0);
    
  else
    error('No transfer function or state space model')
  end
  
end
