
function [Y, T, X, U] = kf_qmpc()

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
    [Ad, Bd, Ed, Cd] = DesignDiscreteMatrices(A, B, E, C, Ts);

    % Design kalman gain and add integral action to the matrices
    pkg load control
    pkg load optim
    [Kfx, Ae, Be, Ce, Ee] = DesignKalman(Ad, Bd, Cd, Ed);

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

    % Create MPC matrices
    Phi = PhiMat(Ae, Ce, N)
    Gamma = GammaMat(Ae, Be, Ce, N)
    Gammad = GammaMat(Ae, Ee, Ce, N)
    barH = barHMat(Gamma, Qz, S, N, nu, nz, Seta)
    Mx0 = Mx0Mat(Gamma, QZ, Phi)
    Mum1 = Mum1Mat(N, nu, S)
    MR = MRMat(Gamma, QZ)
    MD = MDMat(Gamma, QZ, Gammad)
    Lambda = LambdaMat(N, nu)
    barseta = barsetaVec(seta, N)

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

function barH = barHMat(Gamma, Qz, S, N, nu, nz, Seta)
  QZ = QZMat(Qz, N, nz);
  HS = HSMat(S, N, nu);
  barSeta = barSetaMat(Seta, N, nu);
  H = Gamma' * QZ * Gamma + HS;
  z = zeros(nu * N, nu * N);
  barH = [H z; z barSeta];
end

function barSeta = barSetaMat(Seta, N, nu)
  barSeta = eye(nu * N) * Seta;
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

function barseta = barsetaVec(seta, N)
  barseta = ones(N, 1) * seta;
end

function gbar = gbarVec(Mx0, x0, MR, R, D, Mum1, dum1, barseta)
  g = Mx0 * x0 + MR * R + MD * D + Mum1 * dum1;
  gbar = [g; barseta];
end

function R = RVec(r, N)
  R = repmat(r, N, 1);
end

function D = DVec(d, N)
  D = repmat(d, N, 1);
end

function Umin = UminVec(umin, N)
  Umin = repmat(umin, N, 1);
end

function Umax = UmaxVec(umax, N)
  Umax = repmat(umax, N, 1);
end

function Udmin = UdminVec(udmin, N)
  Udmin = repmat(udmin, N-1, 1);
end

function Udmax = UdmaxVec(udmax, N)
  Udmax = repmat(udmax, N-1, 1);
end

function Zmax = ZmaxVec(zmax, N)
  Zmax = repmat(zmax, N, 1);
end

function Zmin = ZminVec(zmin, N)
  Zmin = repmat(zmin, N, 1);
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
