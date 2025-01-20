% Use Model Predictive Control with integral action, quadratic programming and kalman-bucy filter
% Input: sys(Continuous state space model), N(Horizon number), r(Reference vector), umin(Minimum input vector),
% umax(Maximum input vector), zmin(Minimum output vector), zmax(Maximum output vector),
% deltaumin(Minimum output vector rate of change), deltaumax(Maximum rate of change output vector),
% Ts(The sample time, optional), T(End time, optional), x0(Initial state, optional)
% s(Regularization value, optional), Qz(Weight parameter, optional), qw(Disturbance kalman filter tuning, optional)
% rv(Noice kalman filter tuning, optional), Spsi(Slack variable matrix tuning, optional), spsi(Slack variable vector tuning, optional)
% d(Disturbance vector e.g other measurements rather than y, optional), E(Disturbance input signal matrix, optional)
% Output: y(Output signal), T(Discrete time vector), X(State vector), U(Output signal)
% Example 2: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax)
% Example 3: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, Ts)
% Example 4: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, Ts, T)
% Example 5: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, Ts, T, x0)
% Example 6: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, Ts, T, x0, s)
% Example 7: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, Ts, T, x0, s, Qz)
% Example 8: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, Ts, T, x0, s, Qz, qw)
% Example 9: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, Ts, T, x0, s, Qz, qw, rv)
% Example 10: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, Ts, T, x0, s, Qz, qw, rv, Spsi)
% Example 11: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, Ts, T, x0, s, Qz, qw, rv, Spsi, spsi)
% Example 12: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, Ts, T, x0, s, Qz, qw, rv, Spsi, spsi, d)
% Example 13: [Y, T, X, U] = mc.kf_qmpc(sys, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, Ts, T, x0, s, Qz, qw, rv, Spsi, spsi, d, E)
% Author: Daniel Mårtensson 2025 Januari 20

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
    deltaumin = varargin{8};
  else
    error('Missing deltaumin');
  end

  % Get udmax
  if(length(varargin) >= 9)
    deltaumax = varargin{9};
  else
    error('Missing deltaumax');
  end

  % Get sample time
  if(length(varargin) >= 10)
    Ts = varargin{10};
  else
    Ts = 1;
  end

  % Get time
  if(length(varargin) >= 11)
    T = varargin{11};
  else
    T = 10;
  end

  % Get initial state x
  if(length(varargin) >= 12)
    x = varargin{12};
  else
    x = 0;
  end

  % Get regularization parameter s
  if(length(varargin) >= 13)
    s = varargin{13};
  else
    s = 1;
  end

  % Get weight parameter Qz
  if(length(varargin) >= 14)
    Qz = varargin{14};
  else
    Qz = 1;
  end

  % Get kalman disturbance qw
  if(length(varargin) >= 15)
    qw = varargin{15};
  else
    qw = 1;
  end

  % Get kalman noise rv
  if(length(varargin) >= 16)
    rv = varargin{16};
  else
    rv = 1;
  end

  % Get slack parameter Spsi
  if(length(varargin) >= 17)
    Spsi = varargin{17};
  else
    Spsi = 1;
  end

  % Get slack parameter spsi
  if(length(varargin) >= 18)
    spsi = varargin{18};
  else
    spsi = 1;
  end

  % Get disturbance
  if(length(varargin) >= 19)
    d = varargin{19};
  else
    d = 0;
  end

  % Get disturbance matrix E
  if(length(varargin) >= 20)
    E = varargin{20};
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

    % This is the infinity value for single precision
    infinity = realmax('float');

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

    % Create the kalman gain matrix K - Here we use Kalman-Bucy (1961) filter instead of Kalman Filter (1960).
    syse = mc.ss(delay, Ad, Bd, Cd);
    syse.sampleTime = Ts;
    Qw = qw * eye(nx); % old size is nx + nu
    Rv = rv * eye(nz);
    [K] = mc.lqe(syse, Qw, Rv);

    % Create the Phi matrix and lower hankel toeplitz Gamma matrix of inputs - Equation (3.6)
    Phi = PhiMat(Ad, Cd, N);
    Gamma = GammaMat(Ad, Bd, Cd, N);

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
    Gammad = GammaMat(Ad, Ed, Cd, N);
    D = DVec(d, N);

    % Create the QP solver matrix for the gradient - Equation (3.32)
    Mx0 = Mx0Mat(Gamma, QZ, Phi);
    Mum1 = Mum1Mat(N, nu, S);
    MR = MRMat(Gamma, QZ);
    MD = MDMat(Gamma, QZ, Gammad);

    % Create constraints on the movment - Equation (3.38)
    deltaUmin = deltaUminVec(deltaumin, N);
    deltaUmax = deltaUmaxVec(deltaumax, N);
    Lambda = LambdaMat(N, nu);

    % Create constraints on outputs - Equation (3.43)
    Zmin = ZminVec(zmin, N);
    Zmax = ZmaxVec(zmax, N);

    % Create the slack variables - Equation (3.49)
    barSpsi = barSpsiMat(Spsi, N, nu);
    barspsi = barspsiVec(spsi, N);

    % Create QP solver matrix - Equation (3.51)
    barH = barHMat(H, barSpsi, N, nu);

    % Create time vector
    t = 0:Ts:T;
    L = length(t);

    % Create outputs for the simulation
    u = zeros(nu, 1);
    y = zeros(nz, 1);
    um1 = zeros(nu, 1);
    eta = zeros(nz, 1);

    % Create measurement noise
    v = rv * randn(nz, L);

    % Copy the state
    x0 = x;

    % Create the output vectors
    X = zeros(nx, L);
    U = zeros(nu, L);
    Y = zeros(nz, L);

    % Check if it's MATLAB or Octave
    isOctave = exist('OCTAVE_VERSION', 'builtin') ~= 0;

    % Simulate the MPC
    for k = 1:L
      % Return state, input and output
      X(:,k) = x;
      U(:,k) = u;
      Y(:,k) = y;

      % Give the old u to um1
      um1 = u;

      % Get the disturbance
      disturbance = D(1:nd);

      % Integral action - Equation (3.66)
      psi = r - y;
      eta = eta + psi;

      % Compute candidate state x0 with optimized u and integral action - Equation (3.65)
      x0 = Ad*x + Bd*(u + eta) + Ed*disturbance;

      % Create gradient g - Equation (3.32)
      g = gVec(Mx0, x0, MR, R, MD, D, Mum1, um1);

      % Create constraints on inputs - Equation (3.40)
      Umin = UminVec(umin, deltaumin, um1, N, nu);
      Umax = UmaxVec(umax, deltaumax, um1, N, nu);

      % Create constraints for the output - Equation (3.44)
      barZmin = barZminVec(Zmin, Phi, x0, Gammad, D);
      barZmax = barZmaxVec(Zmax, Phi, x0, Gammad, D);

      % Create gradient bar g - Equation (3.51)
      barg = bargVec(g, barspsi);

      % Create barUmin and barUmax - Equation (3.52)
      barUmin = barUminVec(Umin, N);
      barUmax = barUmaxVec(Umax, infinity, N);
      UI = eye(2 * N);

      % Create bmin, bmax and A - Equation (3.56)
      bmin = bminVec(deltaUmin, barZmin, infinity, N);
      bmax = bmaxVec(deltaUmax, barZmax, infinity, N);
      A = AMat(Lambda, Gamma, N);

      % Create for QP - Equation (3.57)
      % barUmin <= I*U <= barUmax
      % bmin <= A*U <= bmax
      aqp = [UI; A; -UI; -A];
      bqp = [barUmax; bmax; -barUmin; -bmin];

      % Quadratic programming for propotional action for u
      if(isOctave == 1)
        [output, ~, e] = qp ([], barH, barg, [], [], [], [], [], aqp, bqp);
        if(e.info == 3)
          error('Quadratic programming QP could not optimize input signals. Try increase the horizion N number.');
        end
      else
        [output, solution] = mc.quadprog(barH, barg, aqp, bqp); % Used for MATLAB users
        if(solution == false)
          error('Quadratic programming quadprog could not optimize input signals. Try to decrease the horizion N number or remove/change lambda regularization. Perhaps increase the slack variable.');
        end
      end

      % Set the input signal
      u = output(1:nu);

      % Compute outputs and states - Equation (3.67)
      y = Cd*x + v(:, k)*0;

      % Compute model with the optimized u - Equation (3.65)
      x = Ad*x + Bd*u + Ed*disturbance;

      % Update error - Equation (3.72)
      e = y - Cd*x;

      % Kalman update - Equation (3.75)
      x = x + K*e;

    end

    %Cange t and y vector and u so the plot look like it is discrete - Important!
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
      if (Ts > 0)
        xlabel(strcat(num2str(Ts), ' time unit/sample'));
      else
        xlabel('Time units');
      end
      grid on
    end
    title('Model Predictive Control With Integral Action And Quadratic Programming')
  else
    % TF to SS
    sys = mc.tf2ss(sys, 'OCF');
    [Y, T, X, U] = kf_qmpc(sys, x0, R, N, umin, umax, udmin, udmax, zmin, zmax);
  end
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

function barH = barHMat(H, barSpsi, N, nu)
  z = zeros(nu * N, nu * N);
  barH = [H z; z barSpsi];
end

function barSpsi = barSpsiMat(Seta, N, nu)
  barSpsi = eye(nu * N) * Seta;
end

function A = AMat(Lambda, Gamma, N)
  A = [Lambda zeros(N-1, N); Gamma -eye(N, N); Gamma eye(N, N)];
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

function barg = bargVec(g, barspsi)
  barg = [g; barspsi];
end

function g = gVec(Mx0, x0, MR, R, MD, D, Mum1, um1)
  g = Mx0 * x0 + MR * R + MD * D + Mum1 * um1;
end

function R = RVec(r, N)
  R = repmat(r, N/length(r), 1);
end

function D = DVec(d, N)
  D = repmat(d, N/length(d), 1);
end

function Umin = UminVec(umin, deltaumin, um1, N, nu);
  Umin = repmat(umin, N/length(umin), 1);
  Umin(1:nu) = max(umin, deltaumin + um1);
end

function Umax = UmaxVec(umax, deltaumax, um1, N, nu);
  Umax = repmat(umax, N/length(umax), 1);
  Umax(1:nu) = min(umax, deltaumax + um1);
end

function deltaUmin = deltaUminVec(deltaumin, N)
  deltaUmin = repmat(deltaumin, (N-1)/length(deltaumin), 1);
end

function deltaUmax = deltaUmaxVec(deltaumax, N)
  deltaUmax = repmat(deltaumax, (N-1)/length(deltaumax), 1);
end

function Zmax = ZmaxVec(zmax, N)
  Zmax = repmat(zmax, N/length(zmax), 1);
end

function Zmin = ZminVec(zmin, N)
  Zmin = repmat(zmin, N/length(zmin), 1);
end

function barZmin = barZminVec(Zmin, Phi, x0, Gammad, D)
  barZmin = Zmin - Phi * x0 - Gammad * D;
end

function barZmax = barZmaxVec(Zmax, Phi, x0, Gammad, D)
  barZmax = Zmax - Phi * x0 - Gammad * D;
end

function barUmin = barUminVec(Umin, N)
  barUmin = [Umin; zeros(N, 1)];
end

function barUmax = barUmaxVec(Umax, infinity, N)
  barUmax = [Umax; infinity * ones(N, 1)];
end

function bmin = bminVec(deltaUmin, barZmin, infinity, N)
  bmin = [deltaUmin; -infinity * ones(N, 1); barZmin];
end

function bmax = bmaxVec(deltaUmax, barZax, infinity, N)
  bmax = [deltaUmax; barZax; infinity * ones(N, 1)];
end

function [Ad, Bd, Cd, Ed] = DiscreteMatrices(A, B, C, E, Ts)
  % Discrete matrices for fixed step based simulation
  nx = size(A, 1);
  nu = size(B, 2);
  nd = size(E, 2);
  M1 = [A B E; zeros(nu + nd, nx + nu + nd)];
  M2 = expm(M1 * Ts);
  Ad = M2(1:nx, 1:nx);
  Bd = M2(1:nx, nx + 1:nx + nu);
  Ed = M2(1:nx, nx + nu + 1:nx + nu + nd);
  Cd = C;
end
