D = 0;
sys = mc.ss(0, [D 1; -1 -1], [0;1], [1 0]); % SISO state space model

sysd = mc.c2d(sys, 0.5); % To discrete

R = 6; % Reference for the SISO model. If MIMO -> R need to be a vector
N = 20; % Horizon predict 
T = 15; % Horizon time
lambda =7; % Regularization for smoother inputs u

[y, t, x, u] = mc.lmpc(sysd, N, R, T, lambda); % Simulate M?C with linear programming 

figure(2); % New figure
[y, t, x, u] = mc.qmpc(sysd, N, R, T, lambda); % Simulate M?C with quadratic programming
