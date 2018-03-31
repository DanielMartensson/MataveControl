% Generates the model predicitve state feedback state space model with disturbance matrix Bd and noise matrix Bn
% Input: SS, Np, Nc, Rw, K, Bd(optional), Bn(optional)
% Example 1: [regsys, L, Kr] = mpcreg(sys, Np, Nc, Rw, K)
% Example 2: [regsys, L, Kr] = mpcreg(sys, Np, Nc, Rw, K, Bd)
% Example 3: [regsys, L, Kr] = mpcreg(sys, Np, Nc, Rw, K, Bd, Bn)
% Author: Daniel Mårtensson, Februari 2018

function [regsys, L, Kr] = mpcreg(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error ('Missing input')
  end
  % Get model type
  type = varargin{1}.type;  
  % Check if there is a TF or SS model
  if(strcmp(type, 'SS' ))
  
    % Get the predict horizon Np
    if(length(varargin) >= 2)
      Np = varargin{2};
    else
      error('Missing the predict horizon Np');
    end
    
    % Get the control horizon Nc
    if(length(varargin) >= 3)
      Nc = varargin{3};
    else
      error('Missing the control horizon Nc');
    end
    
    % Get the tuning parameter Rw
    if(length(varargin) >= 4)
      Rw = varargin{4};
    else
      error('Missing the tuning parameter Rw');
    end
    
    % Get the kalman filter gain matrix K
    if(length(varargin) >= 5)
      K = varargin{5};
    else
      error('Missing the kalman gain matrix K');
    end
    
    % Get the disturbance matrix Bd
    if(length(varargin) >= 6)
      Bd = varargin{6};
    end
    
    % Get the noise matrix Bn
    if(length(varargin) >= 7)
      Bn = varargin{7};
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
    
    % Check if the model is discrete! MPC can only be discrete in this case.
    if sampleTime <= 0
      error('Only discrete state space models')
    end
    
    % Before we entering the switch case, we need to find 
    % the MPC control L and precompensator factor Kr
    
    % Compute the F matrix now!
    F = Fmatrix(C, A, Np);
    % Compute the PHI matrix now
    PHI = PHImatrix(C, A, B, Np, Nc);
    % Find the MPC control law L
    barR = Rw*eye(Nc, Nc);
    L = inv(PHI'*PHI+barR)*PHI'*F;
    % Get size of B matrix
    [n, m] = size(B);
    % Get the m rows of L control law
    L = L(m, :);
    % Find the MPC precompensator factor Kr
    barRs = ones(1, Np)';
    Kr = inv(PHI'*PHI+barR)*PHI'*barRs;
    % Get the m rows
    Kr = Kr(m, :);
    
    % Create new feedback model
    switch regulatorNumber
      case 5 % MPC - Without disturbance matrix and noise matrix
        % Create the A matrix
        A11 = (A-B*L);
        A12 = B*L;
        A22 = (A-K*C);
        A21 = zeros(size(A11, 2), size(A22, 1));
        A = [A11 A12; A21 A22];
        
        % Now create B matrix with precompensator factor - For better tracking
        B11 = B*Kr;
        B21 = zeros(size(B));
        B = [B11; B21];
        
        % Create the C matrix
        C11 = C;
        C12 = C*0;
        C21 = C;
        C22 = C*0;
        C31 = -L;
        C32 = L;
        C = [C11 C12; C21 C22; C31 C32];
        
        % Create D matrix
        D11 = zeros(size(C11,1), size(B11, 2));
        D21 = zeros(size(C11,1), size(B11, 2));
        D31 = Kr;
        D = [D11; D21; D31];
        
        regsys = ss(delay, A, B, C, D);
        regsys.sampleTime = sampleTime;
        
      case 6 % MPC - Without noise matrix
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

        % Now create B matrix with precompensator factor - For better tracking
        B11 = B*Kr;
        B21 = zeros(size(Bd,1), size(B, 2));
        B12 = Bd;
        B22 = Bd;
        B = [B11 B12 ; B21 B22];
        
        % Create the C matrix
        C11 = C;
        C12 = C*0;
        C21 = C;
        C22 = C*0;
        C31 = -L;
        C32 = L;
        C = [C11 C12; C21 C22; C31 C32];
        
        % Create D matrix
        D11 = zeros(size(C11,1), size(B11, 2));
        D21 = zeros(size(C11,1), size(B11, 2));
        D31 = Kr;
        D12 = zeros(size(C11, 1), size(B12, 2));
        D22 = zeros(size(C11, 1), size(B12, 2));
        D32 = zeros(size(C31, 1), size(B12, 2));
        D = [D11 D12; D21 D22; D31 D32];
        
        regsys = ss(delay, A, B, C, D);
        regsys.sampleTime = sampleTime;
      case 7 % MPC 
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
        
        % Now create B matrix with precompensator factor Kr - For better tracking
        B11 = B*Kr;
        B21 = zeros(size(Bd,1), size(B, 2));
        B12 = Bd;
        B22 = Bd;
        B23 = -K*Bn;
        B13 = zeros(size(Bd, 1), size(B23, 2));
        B = [B11 B12 B13; B21 B22 B23];
        
        % Create the C matrix
        C11 = C;
        C12 = C*0;
        C21 = C;
        C22 = C*0;
        C31 = -L;
        C32 = L;
        C = [C11 C12; C21 C22; C31 C32];
        
        % Create D matrix
        D11 = zeros(size(C11,1), size(B11, 2));
        D21 = zeros(size(C11,1), size(B11, 2));
        D31 = Kr;
        D12 = zeros(size(C11, 1), size(B12, 2));
        D22 = zeros(size(C11, 1), size(B12, 2));
        D32 = zeros(size(C31, 1), size(B12, 2));
        D13 = zeros(size(C11, 1), size(B23, 2));
        D23 = Bn;
        D33 = zeros(size(C32, 1), size(B23, 2));
        D = [D11 D12 D13; D21 D22 D23; D31 D32 D33];

        regsys = ss(delay, A, B, C, D);
        regsys.sampleTime = sampleTime;
     end
    
  elseif(strcmp(type, 'TF' ))
    disp('Only state space models only')
  else
    error('This is not TF or SS');
  end
end

function [F] = Fmatrix(C, A, Np)
  F = [];
  for i = 1:(Np)
    F = [F; C*A^i];
  end
  
end

function [PHI] = PHImatrix(C, A, B, Np, Nc)
  F = [];
  PHI = [];
  for j = 1:Nc
    for i = (1-j):(Np-j)
      
      if i < 0
        F = [F; 0*C*A^i*B];
      else
        F = [F; C*A^i*B];
      end
      
    end
    
    % Add to PHI
    PHI = [PHI F];
    % Clear F
    F = [];
  end
  
end

