
function [Y, T, X, U] = kf_qmpc(varargin)
    % Check if there is any input
    if(isempty(varargin))
      error('Missing inputs')
    end

    % Get model
    if(length(varargin) >= 1)
      sys = varargin{1};
    else
      error('Missing model');
    end

    % Get horizon
    if(length(varargin) >= 2)
      N = varargin{2};
    else
      error('Missing horizon');
    end

    % Get reference
    if(length(varargin) >= 3)
      r = varargin{3};
    else
      error('Missing reference');
    end

    % Get umin
    if(length(varargin) >= 4)
      umin = varargin{4};
    else
      error('Missing umin');
    end

    % Get umax
    if(length(varargin) >= 5)
      umax = varargin{5};
    else
      error('Missing umax');
    end

    % Get zmin
    if(length(varargin) >= 6)
      zmin = varargin{6};
    else
      error('Missing zmin');
    end

    % Get zmax
    if(length(varargin) >= 7)
      zmax = varargin{7};
    else
      error('Missing zmax');
    end

    % Get udmin
    if(length(varargin) >= 8)
      udmin = varargin{8};
    else
      error('Missing udmin');
    end

    % Get udmax
    if(length(varargin) >= 9)
      udmax = varargin{9};
    else
      error('Missing udmax');
    end

    % Get initial state x0
    if(length(varargin) >= 10)
      x0 = varargin{10};
    else
      x0 = 0;
    end

    % Get regularization parameter s
    if(length(varargin) >= 11)
      s = varargin{11};
    else
      s = 1;
    end

    % Get weight parameter Qz
    if(length(varargin) >= 12)
      Qz = varargin{12};
    else
      Qz = 1;
    end

    % Get slack parameter Spsi
    if(length(varargin) >= 13)
      Spsi = varargin{13};
    else
      Spsi = 1;
    end

    % Get slack parameter spsi
    if(length(varargin) >= 14)
      spsi = varargin{14};
    else
      spsi = 1;
    end

    % Get disturbance
    if(length(varargin) >= 15)
      d = varargin{15};
    else
      D = 0;
    end

    % Get disturbance matrix E
    if(length(varargin) >= 16)
      E = varargin{16};
    else
      E = 0;
    end

    % Get model type
    type = varargin{1}.type;
    % Check if there is a TF or SS model
    if(strcmp(type, 'SS' ))
      % Get A, B, C, D matrecies
      A = sys.A;
      B = sys.B;
      C = sys.C;
      D = sys.D;
      delay = sys.delay;
      Ts = sys.sampleTime;

      % Check if the model is discrete! MPC can only be discrete in this case.
      if(Ts <= 0)
        error('Only discrete state space models');
      end

      % Get all the sizes
      nx = size(A, 1); % Total number of states from system matrix A[nx * nx]
      nu = size(B, 2); % Total number of inputs from control matrix B[nx * nu]
      nz = size(C, 1); % Total number of outputs from observation matrix C[nz * nx]
      nd = size(E, 2); % Total number of disturbances from disturbance matrix E[nx * nd]

      % The disturbance matrix E need to have the same dimension as system matrix A
      if(size(E, 1) ~= nx)
        E = zeros(nx, nd);
      end

      % Create the discrete matrices - Equation (2.9)
      [Ad, Bd, Cd, Ed] = DiscreteMatrices(A, B, C, E, Ts);

      % Create the augmented matrices - Equation (3.68) and (3.69)
      [Ae, Be, Ce, Ee] = ExtendedMatrices(Ad, Bd, Cd, Ed);

      % Create the Phi matrix and lower hankel toeplitz Gamma matrix of inputs - Equation (3.6)
      Phi = PhiMat(Ae, Ce, N);
      Gamma = GammaMat(Ae, Be, Ce, N);

      % Create reference vector - Equation (3.8)
      R = RVec(r, N);

      % Create the weigth matrix - Equation (3.10)
      QZ = QZMat(Qz, N, nz);

      % Create the regularization matrix - Equation (3.21)
      S = SMat(s, nu);
      HS = HSMat(S, N, nu);

      % Create the QP solver H matrix - Equation (3.24)
      H = HMat(Gamma, QZ, HS);

      % Create the lower hankel toeplitz Gamma matrix of disturbance and its disturbance vector - Equation (3.27)
      Gammad = GammaMat(Ae, Ee, Ce, N);
      D = DVec(d, N);

      % Create the QP solver matrix for the gradient - Equation (3.32)
      Mx0 = Mx0Mat(Gamma, QZ, Phi);
      Mum1 = Mum1Mat(N, nu, S);
      MR = MRMat(Gamma, QZ);
      MD = MDMat(Gamma, QZ, Gammad);

      % Create constraints on the movment - Equation (3.38)
      Udmin = UdminVec(udmin, N);
      Udmax = UdmaxVec(udmax, N);
      Lambda = LambdaMat(N, nu);

      % Create constraints on inputs - Equation (3.40)
      Umin = UminVec(umin, N);
      Umax = UmaxVec(umax, N);

      % Create constraints on outputs - Equation (3.43)
      Zmin = ZminVec(zmin, N);
      Zmax = ZmaxVec(zmax, N);

      % Create the slack variables - Equation (3.49)
      barSpsi = barSpsiMat(Spsi, N, nu);
      barspsi = barspsiVec(spsi, N);

      % Create QP solver matrix - Equation (3.51)
      barH = barHMat(H, N, nu);




    else
      % TF to SS
      sys = mc.tf2ss(sys, 'OCF');
      [Y, T, X, U] = kf_qmpc(sys, x0, R, N, umin, umax, udmin, udmax, zmin, zmax);
    end

    return

    % Model
    A = [0 1; -2 -3];
    B = [0 0; 1 1];
    C = [1 0; 1 0];
    E = [0 0; 2.5 1];
    Ts = 0.5;

    % Parameters
    N = 5;
    big = 10^10;
    t = 0:Ts:30;
    k = 1;
    y = [5];
    un = 3;
    R = 1;
    D = 1;

    % Constraints
    umin = 0;
    umax = 100;
    udmin = 0;
    udmax = 10;
    zmin = 0;
    zmax = 50;
    [Umin, Umax, Udmin, Udmax, Zmin, Zmax, ubarmin, ubarmax] = DesignConstraints(umin, umax, udmax, udmin, zmin, zmax, big, N);

    % Turn them discrete
    [Ad, Bd, Ed, Cd] = DesignDiscreteMatrices(A, B, E, C, Ts)

    % Design kalman gain and add integral action to the matrices
    pkg load control
    pkg load optim
    [Kfx, Ae, Be, Ce, Ee] = DesignKalman(Ad, Bd, Cd, Ed)

    % Sizes
    Qz = eye(1);
    S = eye(size(B, 2));
    Seta = eye(1);
    seta = eye(1);
    nx = size(Ae, 2);
    nu = size(Be, 2);
    nd = size(Ee, 2);
    nz = size(Ce, 1);
    QZ = QZMat(Qz, N, nz);
    [barH, Gamma, Gammad, Phi, Mx0, Mum1, MR, MD, Lambda, barseta] = DesignMPCMatrices(Ae, Be, Ee, Ce, Qz, S, N, Seta, seta)



    return
    % Create MPC vectors
    D = DVec(d, N);

    % Compute constraints
    c = Phi * x0 + Gammad * D;
    Zbarmin = Zmin - c;
    Zbarmax = Zmax - c;


    bmin = [dUmin; -big * ones(N, 1); Zbarmin];
    bmax = [dUmax; Zbarmax; big * ones(N, 1)];
    Abar = [Lambda, zeros(N-1, N); Gamma, -eye(N,N); Gamma, eye(N,N)];
    Abar2 = [Abar; -Abar];
    bbar = [bmax; -bmin];

    return

    % Check if it's MATLAB or Octave
    isOctave = exist('OCTAVE_VERSION', 'builtin') ~= 0;

    % Loop and save the data
    for k = 1:length(t)
      % Return states and input signals
      X(:,k) = x;
      U(:,k) = delta; % First element!

      % Compute outputs
      Y(:,k) = C*x + D*delta; % size(B, 2) = If multiple inputs...



    end

    % Change t and y vector and u so the plot look like it is discrete - Important!
    for(i = 1:2:length(Y)*2)
      leftPart = Y(:,1:i);
      rightPart = Y(:,(i+1):end);
      Y = [leftPart Y(:,i) rightPart];
    end

    for(i = 1:2:length(t)*2)
      leftPart = t(1:i);
      rightPart = t((i+1):end);
      t = [leftPart t(i) rightPart];
    end

    for(i = 1:2:length(U)*2)
      leftPart = U(:,1:i);
      rightPart = U(:,(i+1):end);
      U = [leftPart U(:,i) rightPart];
    end

    for(i = 1:2:length(X)*2)
      leftPart = X(:,1:i);
      rightPart = X(:,(i+1):end);
      X = [leftPart X(:,i) rightPart];
    end

    % Just remove the first one
    T = t(:,2:length(t));
    % And the last one
    Y = Y(:,1:(length(Y)-1));
    % And for U and X too
    U = U(:,1:(length(U)-1));
    X = X(:,1:(length(X)-1));
    % Now we have two vectors which look like a discrete signal

    % Plot - How many subplots?
    for i = 1:size(C,1)
      subplot(size(C,1),1,i)
      plot(T, Y(i,:));
      ylabel(strcat('y', num2str(i)));
      if (sampleTime > 0)
        xlabel(strcat(num2str(sampleTime), ' time unit/sample'));
      else
        xlabel('Time units');
      end
      grid on
    end
  title('Model Predictive Control With Integral Action And Quadratic Programming')
end

function Phi = PhiMat(A, C, N)

  % Create the special Observabillity matrix
  Phi = [];
  for i = 1:N
    Phi = vertcat(Phi, C*A^i);
  end

end

function Gammad = GammadMat(A, B, E, N)
  Gammad = GammaMat(A, B, E, N);
end

function Gamma = GammaMat(A, B, C, N)

  % Create the lower triangular toeplitz matrix
  Gamma = [];
  for i = 1:N
    Gamma = horzcat(Gamma, vertcat(zeros((i-1)*size(C*A*B, 1), size(C*A*B, 2)),cabMat(A, B, C, N-i+1)));
  end

end

function CAB = cabMat(A, B, C, N)

  % Create the column for the Gamma matrix
  CAB = [];
  for i = 0:N-1
    CAB = vertcat(CAB, C*A^i*B);
  end

end

function Mx0 = Mx0Mat(Gamma, QZ, Phi)
  Mx0 = Gamma' * QZ * Phi;
end

function MR = MRMat(Gamma, QZ)
  MR = -Gamma' * QZ;
end

function MD = MDMat(Gamma, QZ, Gammad)
  MD = Gamma' * QZ * Gammad;
end

function Mum1 = Mum1Mat(N, nu, S)
  Mum1 = -[S; zeros((N - 1) * nu, nu)];
end

function QZ = QZMat(Qz, N, nz)
  QZ = zeros(N * nz, N * nz);
  kz = 0;
  for k = 1:N
    QZ(kz + 1:kz + nz, kz + 1:kz + nz) = Qz;
    kz = kz + nz;
  end
end

function Lambda = LambdaMat(N, nu)
  Lambda = zeros((N - 1) * nu, N * nu);
  T = [-eye(nu, nu) eye(nu, nu)];
  for k = 1:N - 1
    Lambda((k - 1) * nu + 1:k * nu, (k - 1) * nu + 1:(k + 1) * nu) = T;
  end
end

function H = HMat(Gamma, QZ, HS)
  H = Gamma' * QZ * Gamma + HS;
end

function S = SMat(s, nu)
  S = s * eye(nu);
end

function barH = barHMat(H, N, nu)
  z = zeros(nu * N, nu * N);
  barH = [H z; z barSeta];
end

function barSpsi = barSpsiMat(Seta, N, nu)
  barSpsi = eye(nu * N) * Seta;
end

function HS = HSMat(S, N, nu)
  HS = zeros(N * nu, N * nu);
  if N == 1
    HS = S;
  else
    k = 0;
    HS(1:nu, 1:nu) = 2 * S;
    HS(1 + nu:nu + nu, 1:nu) = -S;

    for k = 1:N - 2
      ku = k * nu;
      HS(ku-nu+1:ku,ku+1:ku+nu) = -S;
      HS(ku+1:ku+nu,ku+1:ku+nu) =2*S;
      HS(ku+nu+1:ku+2*nu,ku+1:ku+nu) = -S;
    end

    k = N - 1;
    ku = k * nu;
    HS(ku-nu+1:ku,ku+1:ku+nu) = -S;
    HS(ku+1:ku+nu,ku+1:ku+nu) = S;
  end
end

function barspsi = barspsiVec(spsi, N)
  barspsi = spsi * ones(N, 1);
end

function gbar = gbarVec(Mx0, x0, MR, R, D, Mum1, dum1, barseta)
  g = Mx0 * x0 + MR * R + MD * D + Mum1 * dum1;
  gbar = [g; barseta];
end

function R = RVec(r, N)
  R = repmat(r, N/length(r), 1);
end

function D = DVec(d, N)
  D = repmat(d, N/length(d), 1);
end

function Umin = UminVec(umin, N)
  Umin = repmat(umin, N/length(umin), 1);
end

function Umax = UmaxVec(umax, N)
  Umax = repmat(umax, N/length(umax), 1);
end

function Udmin = UdminVec(udmin, N)
  Udmin = repmat(udmin, (N-1)/length(udmin), 1);
end

function Udmax = UdmaxVec(udmax, N)
  Udmax = repmat(udmax, (N-1)/length(udmax), 1);
end

function Zmax = ZmaxVec(zmax, N)
  Zmax = repmat(zmax, N/length(zmax), 1);
end

function Zmin = ZminVec(zmin, N)
  Zmin = repmat(zmin, N/length(zmin), 1);
end

function ubarmin = ubarminvec(Umin, N)
  ubarmin = [Umin; zeros(N, 1)];
end

function ubarmax = ubarmaxvec(Umax, big, N)
  ubarmax = [Umax; ones(N, 1) * big];
end


function [Umin, Umax, Udmin, Udmax, Zmin, Zmax, ubarmin, ubarmax] = DesignConstraints(umin, umax, udmax, udmin, zmin, zmax, big, N)

    Umin = repmat(umin, N, 1);
    Umax = repmat(umax, N, 1);

    Udmin = repmat(udmin, N-1, 1);
    Udmax = repmat(udmax, N-1, 1);

    Zmin = repmat(zmin, N, 1);
    Zmax = repmat(zmax, N, 1);

    ubarmin = [Umin; zeros(N, 1)];
    ubarmax = [Umax; ones(N, 1) * big];

end

function [Ad, Bd, Cd, Ed] = DiscreteMatrices(Ac, B, C, E, Ts)
  % Discrete matrices for fixed step based simulation
  nx = size(A, 1);
  nu = size(B, 2);
  nd = size(E, 2);
  M1 = [A B E; zeros(nu + nd, nx + nu + nd)];
  M2 = expm(M1 * Ts);
  Ad = M2(1:nx, 1:nx);
  Bd = M2(1:nx, nx + 1:nx + nu);
  Ed = M2(1:nx, nx + nu + 1:nx + nu + nd);
  Cd = Cc;
end

function [Ad, Bd, Ed, Cd] = DesignDiscreteMatrices(Ac, Bc, Ec, Cc, Ts)
    nx = size(Ac, 1);
    nu = size(Bc, 2);
    nd = size(Ec, 2);
    M1 = [Ac Bc Ec; zeros(nu + nd, nx + nu + nd)];
    M2 = expm(M1 * Ts);
    Ad = M2(1:size(Ac, 1), 1:size(Ac, 2));
    Bd = M2(1:size(Ac, 1), size(Ac, 2) + 1:size(Ac, 2) + size(Bc, 2));
    Ed = M2(1:size(Ec, 1), size(Ac, 2) + size(Bc, 2) + 1:end);
    Cd = Cc;
end

function [Ae, Be, Ce, Ee] = ExtendedMatrices(A, B, C, E)
  % Extended matrices for integral action
  nx = size(A, 1);
  nz = size(C, 1);
  nu = size(B, 2);
  nd = size(E, 2);
  Ae = [A B; zeros(nu, nx) eye(nu, nu)];
  Be = [B; zeros(nu, nu)];
  Ee = [E; zeros(nu, nd)];
  Ce = [C zeros(nz, nu)];
end


function [Kfx, Ae, Be, Ce, Ee] = DesignKalman(A, B, C, E)
    nx = size(A, 1); % Number of states
    nz = size(C, 1); % Number of measurements
    nu = size(B, 2); % Number of inputs
    nd = size(E, 2); % Number of disturbances

    % Kalman design
    Qw = eye(nx, nx) * 1e-2;   % Process noise covariance
    Qxi = eye(nz, nz) * 1e-2;  % Measurement noise covariance
    Rv = eye(nz, nz);          % Measurement noise covariance
    Ad = eye(nu, nu);          % Identity for disturbance model

    % Extended matrices for Kalman Filter
    Ae = [A B; zeros(nu, nx) Ad];
    Be = [B; zeros(nu, nu)];
    Ee = [E; zeros(nu, nd)];
    Ce = [C zeros(nz, nu)];

    % Covariance matrix for Kalman filter
    Qe = eye(size(Ae)); % [Qw zeros(nx, nz); zeros(nz, nx) Qxi];

    % Solve the Discrete Algebraic Riccati Equation (DARE) for P
    P = dare(Ae', Ce', Qe, Rv);

    % Compute Kalman gain Kfx
    Re = Ce * P * Ce' + Rv;
    Kfx = P * (Ce' / Re);

    %keyboard  % Uncomment this line for debugging if needed
end

function [barH, Gamma, Gammad, Phi, Mx0, Mum1, MR, MD, Lambda, barseta] = DesignMPCMatrices(A, B, E, Cz, Qz, S, N, Seta, seta)

    nx = size(A, 2);
    nu = size(B, 2);
    nd = size(E, 2);
    nz = size(Cz, 1);

    %Form Gamma, Gammad, Phi
    Gamma = zeros(N * nz, N*nu);
    Gammad = zeros(N * nz, N*nd);
    Phi = zeros(N * nz, nx);

    T = Cz;
    kz = 0;
    for k = 1:N
        Gamma(kz+1:kz+nz, 1:nu) = T * B;
        Gammad(kz+1:kz+nz, 1:nd) = T * E;
        T = T * A;
        Phi(kz+1:kz+nz, 1:nx) = T;
        kz = kz + nz;
    end

    for k = 2:N
      Gamma((k - 1) * nz + 1:end, (k - 1) * nu + 1:k * nu) = Gamma(1:(N + 1 - k) * nz, 1:nu);
      Gammad((k - 1) * nz + 1:end, (k - 1) * nd + 1:k * nd) = Gammad(1:(N + 1 - k) * nz, 1:nd);
    end

    %Form QZ
    QZ = zeros(N * nz, N * nz);
    kz = 0;
    for k = 1:N
        QZ(kz + 1:kz + nz, kz + 1:kz + nz) = Qz;
        kz = kz + nz;
    end

    %Form HS
    HS = zeros(N * nu, N * nu);

    if N == 1
        HS = S;
    else
        k = 0;
        HS(1:nu, 1:nu) = 2 * S;
        HS(1 + nu:nu + nu, 1:nu) = -S;

        for k = 1:N - 2
            ku = k * nu;
            HS(ku-nu+1:ku,ku+1:ku+nu) = -S;
            HS(ku+1:ku+nu,ku+1:ku+nu) =2*S;
            HS(ku+nu+1:ku+2*nu,ku+1:ku+nu) = -S;
        end

        k = N - 1;
        ku = k * nu;
        HS(ku-nu+1:ku,ku+1:ku+nu) = -S;
        HS(ku+1:ku+nu,ku+1:ku+nu) = S;
    end

    %Form Mum1
    Mum1 = [-S; zeros((N - 1) * nu, nu)];

    %Form barSeta
    barSeta = eye(N*nu) * Seta;
    barseta = ones(N, 1) * seta;

    %Form H, Mx0, MR, MD
    T = Gamma' * QZ;
    H = T * Gamma + HS;
    H = (H + H') / 2;
    Mx0 = T * Phi;
    MR = -T;
    MD = T * Gammad;

    %barH, barg
    z = zeros(nu*N, nu*N);
    barH = [H z; z barSeta];

    %Form Lambda
    Lambda = zeros((N - 1) * nu, N * nu);
    T = [-eye(nu, nu) eye(nu, nu)];

    for k = 1:N - 1
        Lambda((k - 1) * nu + 1:k * nu, (k - 1) * nu + 1:(k + 1) * nu) = T;
    end
  end

function [u0, xp, info] = MPCCompute(y, um1, R, D, Zmin, Zmax, ...
    ys, zs, us, ds, xp, ubar, ...
    Hbar, Mx0, Mum1, MR, MD, sbareta, ...
    Phi, GammaD, Gamma, Lambda, big, ...
    umin, umax, dumin, dumax, ...
    ubarmin, ubarmax, ...
    dUmin, dUmax, ...
    Kfx, ...
    A, B, E, C, N)

  % Lengths of vectors
  nu = length(um1);
  nd = length(ds);
  nz = length(zs);

  % Form deviation variables
  dy = y - ys;
  dum1 = um1 - us;
  dR = R - repmat(zs, N, 1);
  dD = D - repmat(ds, N, 1);
  dd = dD(1:nd, 1);

  % Kalman Filter
  e = dy - C * xp;
  x0 = xp + Kfx * e;

  % Update gradient
  g = Mx0 * x0 + MR * dR + MD * dD + Mum1 * dum1;
  gbar = [g; sbareta];

  % Create bounds for ubar
  ubarmin(1:nu,1) = max(umin, dumin + dum1);
  ubarmax(1:nu,1) = min(umax, dumax + dum1);

  % Create bounds for Z
  c = Phi * x0 + GammaD * dD;
  Zbarmin = Zmin - c;
  Zbarmax = Zmax - c;

  % Setup matrix system for boundaries
  bmin = [dUmin; -big * ones(N, 1); Zbarmin];
  bmax = [dUmax; Zbarmax; big * ones(N, 1)];
  Abar = [Lambda, zeros(N-1, N); Gamma, -eye(N,N); Gamma, eye(N,N)];

  % Create startguess
  ubarinit = [ubar(nu + 1:N * nu, 1);
              ubar((N - 1) * nu + 1:N * nu, 1);
              ubar(nu * N + nz + 1:end, 1);
              ubar(nu * N + (N - 1) * nz + 1:end)];

 Abar2 = [Abar; -Abar];
 bbar = [bmax; -bmin];

 % Lösning med kvadratisk programmering
 [ubar, feval, EXITFLAG] = quadprog(Hbar, gbar, Abar2, bbar, [], [], ubarmin, ubarmax, ubarinit);

 %Kontroll av om optimeringen var framgångsrik
 if EXITFLAG == 1
     du0 = ubar(1:nu, 1);
     info = 0;
 else
     du0 = dum1;
     info = 1;
 end

  % Update Kalman Filter
  xp = A * x0 + B * du0 + E * dd;

  % Form physical variable
  u0 = du0 + us;

end
