%% This is an oscillating mass-spring-damper system. Classical second order system
 % ---------------------------------------------
 % / / / / / / / / / / / / / / / / / / / / / / /
 % ---------------------------------------------
 %    |                        |
 %    |                        |
 %    |                        |
 %    |                        |
 %    |                        |
 %    |                        |
 %  | | |                     /
 %  | | |                    /
 %  | | |                    \
 %  | | |                     \
 %  | | |                      \
 %  | | |                       \
 %  | | | b = 1.2 [Ns/m]        /  k = 10 [N/m]
 %  |_|_|                      /
 %  |   |                      |
 %  |   |                      |
 %  |_ _|                      |
 %    |                        |
 %    |                        |
 %    |                        |
 %    |                        |
 % ------------------------------------
 % |                                  |
 % |                                  |
 % |             m  = 10 [kg]         |
 % |                                  |---------
 % ------------------------------------        |
 %                 |                           |
 %                 |                           | x [m]
 %                 |                           V
 %                 V
 %                 F [N]
 %
 % System of equation (second order system):
 %                                          m*ddx + b*dx + k*x = F
 %
 % State space (first order system):
 %                                          dx1 = x2
 %                                          dx2 = -x1*k/m - x2*b/m + F/m
 %  [dx1] = [0       1]*[x1] + [0  ]
 %  [dx2] = [-k/m -b/m]*[x2] + [1/m]*F
 %   dx          A       x       B   u
 %


 %% Build the system plant model (The real world system plant model which we don't know)
 m = 10;                      % Weight of the mass                                      (mandatory)
 k = 10;                      % Spring force                                            (mandatory)
 b = 2.2;                     % Damper force                                            (mandatory)
 A = [0 1; -k/m -b/m];        % System matrix                                           (mandatory)
 B = [0; 1/m];                % Input matrix                                            (mandatory)
 C = [2 0];                   % Output matrix                                           (mandatory)
 delay = 0;
 sysp = mc.ss(delay, A, B, C);

 %% Build the system controller model (The estimated real world model which is very similar to the real world system plant model)
 m = 13.5;                    % Weight of the mass                                      (mandatory)
 k = 8.7;                     % Spring force                                            (mandatory)
 b = 3.1;                     % Damper force                                            (mandatory)
 A = [0 1; -k/m -b/m];        % System matrix                                           (mandatory)
 B = [0; 1/m];                % Input matrix                                            (mandatory)
 C = [0.5 0];                 % Output matrix                                           (mandatory)
 delay = 0;
 sysc = mc.ss(delay, A, B, C);

 %% Create the MPC parameters
 N = 20;                      % Control horizon                                         (mandatory)
 r = 10;                      % Reference                                               (mandatory)
 umin = 0;                    % Minimum input value on u                                (mandatory)
 umax = 60;                   % Maximum input value on u                                (mandatory)
 zmin = -1;                   % Minimum output value on y                               (mandatory)
 zmax = 100;                  % Maximum output value on y                               (mandatory)
 deltaumin = -30;             % Minimum rate of change on u                             (mandatory)
 deltaumax = 30;              % Maximum rate of change on u                             (mandatory)
 antiwindup = 100;            % Limits for the 1/s integral                             (mandatory)
 alpha = 0.1;                 % Integral rate                                           (optional)
 Ts_mpc = 10;                 % Sample time for MPC. Total control horizon: Ts_mpc + N  (optional)
 Ts_kf = 1;                   % Sample time for KF (and also plant)                     (optional)
 T = 500;                     % End time for the simulation                             (optional)
 x0 = [0; 0];                 % Intial state for the models sysp and sysc               (optional)
 s = 1;                       % Regularization parameter (Faster to solve QP-problem)   (optional)
 Qz = 1;                      % Weight parameter for QP-solver                          (optional)
 qw = 1;                      % Disturbance tuning for Kalman-Bucy filter               (optional)
 rv = 0.1;                    % Noise tuning for Kalman-Bucy filter                     (optional)
 Spsi_spsi = 1;               % Slackvariable tuning for H matrix and gradient g        (optional)
 d = 3;                       % Disturbance signal at step iteration k = 0              (optional)
 E = [0; 0];                  % Disturbance signal matrix for both sysp and sysc        (optional)

 %% Run MPC with Kalman-bucy filter
 %                                                          d
 %                                                          |
 %                                                          |
 %                        ____________                 _____V_______
 %             + _       |            |               |            |
 %    ---r--|---|_|--R-->|  MPC + KF  |-------u------>|    PLANT   |----------y------>
 %          |   +Λ       |____________|               |____________|          |
 %          |    |                         ___                                |
 %          |    |                        | 1 |      _       _ -              |
 %          |    -------------------------- - |<----|λ|<----|_|<---------------
 %          |                             | s |      -       |+
 %          |                              ---               |
 %          -------------------------------------------------

 [Y, T, X, U] = mc.kf_qmpc(sysp, sysc, N, r, umin, umax, zmin, zmax, deltaumin, deltaumax, antiwindup, alpha, Ts_mpc, Ts_kf, T, x0, s, Qz, qw, rv, Spsi_spsi, d, E);

 % Print the output
 R = r*ones(length(T));
 plot(T, U, T, Y, T, R, 'r--');
 legend('Input', 'Output', 'Reference')
 grid on
