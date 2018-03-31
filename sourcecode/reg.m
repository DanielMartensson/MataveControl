% Generates the state feedback controler with the control law L and with integral action law Li
% Input: sys, L, Li(optional), Kr(optional)
% Example 1: [regsys, Kr] = reg(sys, L)
% Example 2: [regsys, Kr] = reg(sys, L, Li)
% Example 3: [regsys, Kr] = reg(sys, L, Li, Kr)
% Author: Daniel Mårtensson, November 2017

function [regsys, Kr] = reg(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error ('Missing input')
  end
  % Get model type
  type = varargin{1}.type;  
  % Check if there is a TF or SS model
  if(strcmp(type, 'SS' ))
  
    % Get the control law L;
    if(length(varargin) >= 2)
      L = varargin{2};
    else
      error('Missing the control law L');
    end
    
    % Get the integral control law Li
    if(length(varargin) >= 3)
      Li = varargin{3};
    end
    
    % Get the precompensator factor Kr - Reference
    if(length(varargin) >= 4)
      Kr = varargin{4};
    else
      Kr = 0; % For LQI only
    end

    % Check if what feedback controller you should use
    regulatorNumber = length(varargin);
    
    % Get A, B, C, D matrecies
    sys = varargin{1};
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;
    delay = sys.delay;
    sampleTime = sys.sampleTime;

    % Create new feedback model
    switch regulatorNumber
      case 2 % LQR + precompensator for reference
        % Create the A matrix
        A = (A-B*L);
        % Create the B matrix
        % Check if the model is discrete or not
        % Create the precompensator factor for the reference vector
        if sampleTime > 0 
          Kr = 1./(C*inv(eye(size(A)) - A)*B);
        else
          Kr = 1./(C*inv(-A)*B);
        end
        % Now create B matrix with precompensator factor - For better tracking
        B = B*Kr;
        %C matrix and D matrix are the same
        
        regsys = ss(delay, A, B, C, D);
        regsys.sampleTime = sampleTime;
        
      case 3 % LQR with integral action LQI
        % Create A matrix
        A = [(A-B*L) B*Li; (D*L-C) -D*Li];
        
        % Create B matrix
        ny = size(C, 1); % Number outputs
        nu = size(B, 2); % Number inputs
        B = [0*B; ones(ny, nu)]; % <- precompensator for reference = 0
        
        % Create C matrix
        C = [(C-D*L) D*Li];
        
        % Matrix D will be created by it self
        
        regsys = ss(delay, A, B, C, D);
        regsys.sampleTime = sampleTime;
      case 4 % LQR with integral action LQI + precompensator for reference 
        % Create A matrix
        A = [(A-B*L) B*Li; (D*L-C) -D*Li];
        
        % Create B matrix
        ny = size(C, 1); % Number outputs
        nu = size(B, 2); % Number inputs
        B = [Kr*B; ones(ny, nu)]; %<- precompensator for reference = Kr
        
        % Create C matrix
        C = [(C-D*L) D*Li];
        
        % Matrix D will be created by it self
        
        regsys = ss(delay, A, B, C, D);
        regsys.sampleTime = sampleTime;
     end
    
  elseif(strcmp(type, 'TF' ))
    disp('Only state space models')
  else
    error('This is not TF or SS');
  end
end
