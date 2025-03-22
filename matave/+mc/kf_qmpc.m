% Use Model Predictive Control with integral action, quadratic programming and kalman-bucy filter
% This MPC code follows this thesis: https://github.com/DanielMartensson/MataveControl/blob/master/Model%20Predictive%20Control%20for%20an%20artifical%20pancreas%20-%20Matias%20S%C3%B8rensen%20og%20Simon%20Kristiansen.pdf
% Input: sysp(State space model of the plant), sysc(State space model of the controller), N(Horizon number), r(Reference vector),
% umin(Minimum input vector), umax(Maximum input vector), zmin(Minimum output vector), zmax(Maximum output vector),
% deltaumin(Minimum output vector rate of change), deltaumax(Maximum rate of change output vector), antiwindup(Maximum/Minimum value of integral),
% alpha(Integral rate, optional), Ts_mpc(The sample time of MPC, optional), Ts_kf(The sample time of KF, optional), T(End time, optional), x0(Initial state, optional),
% s(Regularization value, optional), Qz(Weight parameter, optional), qw(Disturbance kalman filter tuning, optional),
% rv(Noice kalman filter tuning, optional), Spsi_spsi(Slack variable matrix tuning and slack variable vector tuning, optional),
% d(Disturbance vector e.g other measurements rather than y, optional), E(Disturbance input signal matrix, optional)
% Output: y(Output signal), T(Discrete time vector), X(State vector), U(Output signal)
% Example 1: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup)
% Example 2: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha)
% Example 3: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc)
% Example 4: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc, Ts_kf)
% Example 5: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc, Ts_kf, T)
% Example 6: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc, Ts_kf, T, x0)
% Example 7: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc, Ts_kf, T, x0, s)
% Example 8: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc, Ts_kf, T, x0, s, Qz)
% Example 9: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc, Ts_kf, T, x0, s, Qz, qw)
% Example 10: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc, Ts_kf, T, x0, s, Qz, qw, rv)
% Example 11: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc, Ts_kf, T, x0, s, Qz, qw, rv, Spsi_spsi)
% Example 12: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc, Ts_kf, T, x0, s, Qz, qw, rv, Spsi_spsi, d)
% Example 13: [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc, Ts_kf, T, x0, s, Qz, qw, rv, Spsi_spsi, d, E)
% Author: Daniel Mårtensson 2025 Januari 20
% Update: Add separte sample time for both MPC and KF, 2025-03-19

function [Y, T, X, U] = kf_qmpc(varargin)
  % Check if there is any input
  if(isempty(varargin))
    error('Missing inputs')
  end

  % Get system plant model
  if(length(varargin) >= 1)
    sysp = varargin{1};
  else
    error('Missing system plant model');
  end

  % Get system controller model
  if(length(varargin) >= 2)
    sysc = varargin{2};
  else
    error('Missing system controller model');
  end

  % Get horizon
  if(length(varargin) >= 3)
    N = varargin{3};
  else
    error('Missing horizon');
  end

  % Get reference
  if(length(varargin) >= 4)
    r = varargin{4};
  else
    error('Missing reference');
  end

  % Get umin
  if(length(varargin) >= 5)
    umin = varargin{5};
  else
    error('Missing umin');
  end

  % Get umax
  if(length(varargin) >= 6)
    umax = varargin{6};
  else
    error('Missing umax');
  end

  % Get zmin
  if(length(varargin) >= 7)
    zmin = varargin{7};
  else
    error('Missing zmin');
  end

  % Get zmax
  if(length(varargin) >= 8)
    zmax = varargin{8};
  else
    error('Missing zmax');
  end

  % Get udmin
  if(length(varargin) >= 9)
    deltaumin = varargin{9};
  else
    error('Missing deltaumin');
  end

  % Get udmax
  if(length(varargin) >= 10)
    deltaumax = varargin{10};
  else
    error('Missing deltaumax');
  end

  % Get anti-windup
  if(length(varargin) >= 11)
    antiwindup = varargin{11};
  else
    error('Missing anti-windup');
  end

  % Get integral rate
  if(length(varargin) >= 12)
    alpha = varargin{12};
  else
    alpha = 0.01;
  end

  % Get sample time for MPC
  if(length(varargin) >= 13)
    Ts_mpc = varargin{13};
  else
    Ts_mpc = 1;
  end

  % Get sample time for KF
  if(length(varargin) >= 14)
    Ts_kf = varargin{14};
  else
    Ts_kf = 1;
  end

  % Get time
  if(length(varargin) >= 15)
    T = varargin{15};
  else
    T = 10;
  end

  % Get initial state x
  if(length(varargin) >= 16)
    x = varargin{16};
  else
    x = 0;
  end

  % Get regularization parameter s
  if(length(varargin) >= 17)
    s = varargin{17};
  else
    s = 1;
  end

  % Get weight parameter Qz
  if(length(varargin) >= 18)
    Qz = varargin{18};
  else
    Qz = 1;
  end

  % Get kalman disturbance qw
  if(length(varargin) >= 19)
    qw = varargin{19};
  else
    qw = 1;
  end

  % Get kalman noise rv
  if(length(varargin) >= 20)
    rv = varargin{20};
  else
    rv = 1;
  end

  % Get slack parameter Spsi_spsi
  if(length(varargin) >= 21)
    Spsi_spsi = varargin{21};
  else
    Spsi_spsi = 1;
  end

  % Get disturbance
  if(length(varargin) >= 22)
    d = varargin{22};
  else
    d = 0;
  end

  % Get disturbance matrix E
  if(length(varargin) >= 23)
    E = varargin{23};
    Ep = E;
  else
    E = 0;
    Ep = E;
  end

  % Get model type
  typep = sysp.type;
  typec = sysp.type;

  % Check if there is a TF or SS model
  if(and(strcmp(typep, 'SS' ), strcmp(typec, 'SS' )))
    % Get A, B, C, D matrecies from the plant model
    Ap = sysp.A;
    Bp = sysp.B;
    Cp = sysp.C;
    Dp = sysp.D;
    delayp = sysp.delay;

    % Get A, B, C, D matrecies from the controller model
    A = sysc.A;
    B = sysc.B;
    C = sysc.C;
    D = sysc.D;
    delayc = sysc.delay;

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
    [Ad, Bd, Cd, Ed] = DiscreteMatrices(A, B, C, E, Ts_mpc);
    [Adkf, Bdkf, Cdkf, Edkf] = DiscreteMatrices(A, B, C, E, Ts_kf);
    [Adp, Bdp, Cdp, Edp] = DiscreteMatrices(Ap, Bp, Cp, Ep, Ts_kf);

    % Create the kalman gain matrix K - Here we use Kalman-Bucy (1961) filter instead of Kalman Filter (1960).
    syse = mc.ss(delayc, Adkf, Bdkf, Cdkf);
    syse.sampleTime = Ts_kf;
    Qw = qw * eye(nx);
    Rv = rv * eye(nz);
    [K] = mc.lqe(syse, Qw, Rv); % This is the same as running the MATLAB command dare for computing kalman-bucy gain K

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
    Spsi = Spsi_spsi;
    spsi = Spsi_spsi;
    barSpsi = barSpsiMat(Spsi, N, nu);
    barspsi = barspsiVec(spsi, N, nu);

    % Create QP solver matrix - Equation (3.51)
    barH = barHMat(H, barSpsi, N, nu);

    % Create inequality constraints AA - Equation (3.56)
    AA = AAMat(Lambda, Gamma, N, nu, nz);

    % Create time vector
    t = 0:Ts_kf:T;
    L = length(t);

    % Create outputs for the simulation
    u = zeros(nu, 1);
    y = zeros(nz, 1);
    um1 = zeros(nu, 1);
    eta = zeros(nz, 1);

    % Create measurement noise
    v = rv * randn(nz, L);

    % The plant state must have the same initial state as the controller state x
    xp = x;

    % Get the disturbance
    disturbance = D(1:nd);

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

      % Integral action - Equation (3.66)
      psi = r - y;
      eta = eta + alpha*psi;

      % Limits for the integral eta
      if(abs(eta) > antiwindup)
        eta = sign(eta)*antiwindup;
      end

      % Create gradient g. Also add the integral eta together with reference vector R for adjust the reference settings - Equation (3.32)
      % The reason why adjusting the reference R vector is because then the integral action will be optimized inside the QP-solver.
      Ri = repmat(eta, N, 1);
      g = gVec(Mx0, x, MR, R + Ri, MD, D, Mum1, um1);

      % Create constraints on inputs - Equation (3.40)
      Umin = UminVec(umin, deltaumin, um1, N, nu);
      Umax = UmaxVec(umax, deltaumax, um1, N, nu);

      % Create constraints for the output - Equation (3.44)
      barZmin = barZminVec(Zmin, Phi, x, Gammad, D);
      barZmax = barZmaxVec(Zmax, Phi, x, Gammad, D);

      % Create gradient bar g - Equation (3.51)
      barg = bargVec(g, barspsi);

      % Create barUmin and barUmax - Equation (3.52)
      barUmin = barUminVec(Umin, N);
      barUmax = barUmaxVec(Umax, infinity, N);
      UI = eye(N * nu + N, 2 * N * nu);

      % Create bmin and bmax - Equation (3.56)
      bmin = bminVec(deltaUmin, barZmin, infinity, N, nz);
      bmax = bmaxVec(deltaUmax, barZmax, infinity, N, nz);

      % Create for QP - Equation (3.57)
      % barUmin <= I*U <= barUmax
      % bmin <= AA*U <= bmax
      aqp = [UI; AA; -UI; -AA];
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

      % Compute outputs - Equation (3.67)
      y = Cdp*xp + v(:, k);

      % Compute plant model with the optimized u - Equation (3.65)
      xp = Adp*xp + Bdp*u + Edp*disturbance;

      % Update error - Equation (3.72)
      e = y - Cdkf*x;

      % Kalman update - Equation (3.75)
      x = x + K*e;

      % Compute candidate state x - Equation (3.65)
      x = Adkf*x + Bdkf*u + Edkf*disturbance;
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
      if (Ts_kf > 0)
        xlabel(strcat(num2str(Ts_kf), ' time unit/sample'));
      else
        xlabel('Time units');
      end
      grid on
    end
    title('Model Predictive Control With Integral Action And Quadratic Programming')
  else
    % TF to SS
    if(~strcmp(typep, 'SS' ))
      sysp = mc.tf2ss(sysp, 'OCF');
    end
    if(~strcmp(typec, 'SS' ))
      sysc = mc.tf2ss(sysc, 'OCF');
    end
    [Y, T, X, U] = kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, lambda, Ts_mpc, Ts_kf, T, x0, s, Qz, qw, rv, Spsi_spsi, d, E);
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

function barSpsi = barSpsiMat(Spsi, N, nu)
  barSpsi = Spsi * eye(nu * N);
end

function AA = AAMat(Lambda, Gamma, N, nu, nz)
  AA = [Lambda zeros((N-1)*nu, nu*N); Gamma -eye(nz*N, nu*N); Gamma eye(nz*N, nu*N)];
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

function barspsi = barspsiVec(spsi, N, nu)
  barspsi = spsi * ones(N*nu, 1);
end

function barg = bargVec(g, barspsi)
  barg = [g; barspsi];
end

function g = gVec(Mx0, x0, MR, R, MD, D, Mum1, um1)
  g = Mx0 * x0 + MR * R + MD * D + Mum1 * um1;
end

function R = RVec(r, N)
  R = repmat(r, N, 1);
end

function D = DVec(d, N)
  D = repmat(d, N, 1);
end

function Umin = UminVec(umin, deltaumin, um1, N, nu);
  Umin = repmat(umin, N, 1);
  Umin(1:nu) = max(umin, deltaumin + um1);
end

function Umax = UmaxVec(umax, deltaumax, um1, N, nu);
  Umax = repmat(umax, N, 1);
  Umax(1:nu) = min(umax, deltaumax + um1);
end

function deltaUmin = deltaUminVec(deltaumin, N)
  deltaUmin = repmat(deltaumin, N-1, 1);
end

function deltaUmax = deltaUmaxVec(deltaumax, N)
  deltaUmax = repmat(deltaumax, N-1, 1);
end

function Zmax = ZmaxVec(zmax, N)
  Zmax = repmat(zmax, N, 1);
end

function Zmin = ZminVec(zmin, N)
  Zmin = repmat(zmin, N, 1);
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

function bmin = bminVec(deltaUmin, barZmin, infinity, N, nz)
  bmin = [deltaUmin; -infinity * ones(N*nz, 1); barZmin];
end

function bmax = bmaxVec(deltaUmax, barZmax, infinity, N, nz)
  bmax = [deltaUmax; barZmax; infinity * ones(N*nz, 1)];
end

function [Ad, Bd, Cd, Ed] = DiscreteMatrices(A, B, C, E, Ts)
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
