close all
clear all
clc

sys = mc.ss(0, [0 1; -1 -1], [0;1], [1 0]); % SISO state space model

sysd = mc.c2d(sys, 0.5); % To discrete

R = [6]; % Reference for the SISO model. If MIMO -> R need to be a vector
N = 10; % Horizon predict constant
T = 35; % Horizon time constant
lambda = 7; % Regularization for smoother inputs u

[y, t, x, u] = mc.lmpc(sysd, N, R, T, lambda); % Simulate MPC with linear programming
hold on
plot(t, u)

I = 0.2; % Integral action constant
Umax = [0.4]; % Maximum input signal vector
Ymax = [R + 1]; % We say that the output can be +1 over the reference R
lambda = 0.2; % Regularization for smoother inputs u
figure(2); % New figure
x0 = [-3; 20];
[y, t, x, u] = mc.qmpc(sysd, N, R, T, lambda, Umax, Ymax, I, x0); % Simulate MPC with quadratic programming
hold on
plot(t, u)
