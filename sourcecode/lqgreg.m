% Generates the state feedback state space model with disturbance matrix Bd and noise matrix Bn
% Input: SS, L, K, Bd(optional), Bn(optional)
% Example 1: [regsys] = lqgreg(sys, L, K)
% Example 2: [regsys] = lqgreg(sys, L, K, Bd)
% Example 3: [regsys] = lqgreg(sys, L, K, Bd, Bn)
% Author: Daniel Mårtensson, November 2017

function [regsys] = lqgreg(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error ('Missing input')
  end
  % Get model type
  type = varargin{1}.type;  
  % Check if there is a TF or SS model
  if(strcmp(type, 'SS' ))
    % Get A, B, C, D matrecies
    sys = varargin{1};
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;
    
    % Get the control law L;
    if(length(varargin) >= 2)
      L = varargin{2};
    else
      error('Missing the control law L');
    end
    
    % Get the kalman gain matrix
    if(length(varargin) >= 3)
      K = varargin{3};
    else
      error('Missing the kalman gain matrix K');
    end
    
    % Get the disturbance matrix
    if(length(varargin) >= 4)
      Bd = varargin{4};
    end
    
    % Get the noise matrix
    if(length(varargin) >= 5)
      Bn = varargin{5};
    end
    
    % Check if what feedback controller you should use
    regulatorNumber = length(varargin);
    
    % Get model matrecies
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;
    delay = sys.delay;
    sampleTime = sys.sampleTime;

    % Create new feedback model
    switch regulatorNumber
      case 3 % LQG - Without disturbance matrix and noise matrix
        % Create the A matrix
        A11 = (A-B*L);
        A12 = B*L;
        A22 = (A-K*C);
        A21 = zeros(size(A11, 2), size(A22, 1));
        A = [A11 A12; A21 A22];
        
        % Create the B matrix
        B11 = B;
        B21 = zeros(size(B));
        B = [B11; B21];
        
        % Create the C matrix
        C = [(C-D*L) D*L];
        
        % Create D matrix
        D = D;
        
        regsys = ss(delay, A, B, C, D);
        regsys.sampleTime = sampleTime;
        
      case 4 % LQG - Without noise matrix
        % Create the A matrix
        A11 = (A-B*L);
        A12 = B*L;
        A22 = (A-K*C);
        A21 = zeros(size(A11, 2), size(A22, 1));
        A = [A11 A12; A21 A22];
        
        % Check if Bd have the right dimension
        if(or(size(B,1) ~= size(Bd, 1), size(B,2) ~= size(Bd, 2)))
          str = sprintf('Dustirbance matrix Bd need to have the dimension %ix%i', size(B,1), size(B,2));
          error(str);
        end
        
        % Create B matrix
        B11 = B;
        B21 = zeros(size(Bd,1), size(B, 2));
        B12 = Bd;
        B22 = Bd;
        B = [B11 B12 ; B21 B22];
        
        % Create the C matrix
        C = [(C-D*L) D*L];
        
        % Create D matrix
        D = [D zeros(size(D, 1), size(Bd, 2))];
        
        regsys = ss(delay, A, B, C, D);
        regsys.sampleTime = sampleTime;
      case 5 % LQG 
        % Create the A matrix
        A11 = (A-B*L);
        A12 = B*L;
        A22 = (A-K*C);
        A21 = zeros(size(A11, 2), size(A22, 1));
        A = [A11 A12; A21 A22];
        
        % Check if Bd have the right dimension
        if(size(B,1) ~= size(Bd, 1))
          str = sprintf('Disturbance matrix Bd need to have the dimension %ix%i', size(B,1), 1);
          error(str);
        end
        
        % Check if Bn have the right dimension
        if(size(C,1) ~= size(Bn, 1))
          str = sprintf('Noise matrix Bn need to have the dimension %ix%i', size(C,1), 1);
          error(str);
        end
        
        % Create B matrix
        B11 = B;
        B21 = zeros(size(Bd,1), size(B, 2));
        B12 = Bd;
        B22 = Bd;
        B23 = -K*Bn;
        B13 = zeros(size(Bd, 1), size(B23, 2));
        
        B = [B11 B12 B13; B21 B22 B23];
        
        % Create the C matrix
        C = [(C-D*L) D*L];
        
        % Create D matrix
        D = [D zeros(size(D, 1), size(Bd, 2)) Bn];
        
        regsys = ss(delay, A, B, C, D);
        regsys.sampleTime = sampleTime;
      
     end
    
  elseif(strcmp(type, 'TF' ))
    disp('Only state space models only')
  else
    error('This is not TF or SS');
  end
end
