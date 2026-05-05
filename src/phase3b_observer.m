%% ================================================
%  PHASE 3B — Luenberger Observer Design
%  Purva Ghanate | Servo Control Portfolio Project
%% ================================================
clear; clc; close all;

%% --- Plant Parameters ---
R_m = 1.0;
L   = 0.5;
K   = 0.01;
J   = 0.01;
b   = 0.1;

%% --- State Space ---
A = [-b/J,   K/J;
     -K/L,  -R_m/L];
B = [0; 1/L];
C = [1, 0];
D = 0;

%% ================================================
%  STEP 1 — OBSERVABILITY CHECK
%% ================================================
Ob = obsv(A, C);
rank_Ob = rank(Ob);
fprintf('=== Observability Check ===\n')
fprintf('Observability matrix rank: %d (need %d)\n', rank_Ob, size(A,1))
if rank_Ob == size(A,1)
    fprintf('>> System is OBSERVABLE ✓\n\n')
else
    fprintf('>> NOT observable — observer cannot be designed!\n\n')
end

%% ================================================
%  STEP 2 — CONTROLLER (LQR Balanced from 3A)
%% ================================================
Q  = diag([100, 1]);
R  = 0.01;
K_lqr = lqr(A, B, Q, R);

% Feedforward gain
N = -1 / (C * ((A - B*K_lqr) \ B));

fprintf('=== LQR Gains (Balanced, from Phase 3A) ===\n')
fprintf('K = [%.4f, %.4f] | N = %.4f\n\n', K_lqr(1), K_lqr(2), N)

% Controller closed-loop poles
ctrl_poles = eig(A - B*K_lqr);
fprintf('Controller poles: %.4f%+.4fi, %.4f%+.4fi\n\n',...
    real(ctrl_poles(1)), imag(ctrl_poles(1)),...
    real(ctrl_poles(2)), imag(ctrl_poles(2)))

%% ================================================
%  STEP 3 — OBSERVER POLE PLACEMENT
%% ================================================
% Rule: observer poles ~5x faster than controller poles
% Controller dominant pole real part: ~-16.36
% Target observer poles: ~5x = -80 to -100

% We design 3 observers — slow, medium, fast
% to show the effect of observer speed on estimation

obs_poles_slow   = [-40, -45];       % 2.5x faster than controller
obs_poles_medium = [-80, -85];       % 5x faster  (standard rule)
obs_poles_fast   = [-150, -160];     % 10x faster

% Compute observer gains using pole placement
% Note: observer uses (A', C') — dual of controllability problem
L_slow   = place(A', C', obs_poles_slow)';
L_medium = place(A', C', obs_poles_medium)';
L_fast   = place(A', C', obs_poles_fast)';

fprintf('=== Observer Gains ===\n')
fprintf('Slow   L = [%.4f, %.4f]\n', L_slow(1),   L_slow(2))
fprintf('Medium L = [%.4f, %.4f]\n', L_medium(1), L_medium(2))
fprintf('Fast   L = [%.4f, %.4f]\n\n', L_fast(1), L_fast(2))

%% ================================================
%  STEP 4 — SIMULATE OBSERVER ESTIMATION
%  Show how quickly observer estimates converge
%% ================================================
t  = 0:0.001:1;        % time vector
u  = ones(size(t));    % step voltage input

% True initial state: motor starts at rest
x0_true = [0; 0];

% Observer starts with WRONG initial estimate
% (simulates real scenario — you don't know initial current)
x0_obs_wrong = [0; 5];    % wrong current estimate (5A off)

% Build augmented system for simulation
% Real plant
sys_plant = ss(A, B, C, D);

% Observer dynamics: x̂_dot = (A - L*C)*x̂ + B*u + L*y
% We simulate the estimation error: e = x - x̂
% e_dot = (A - L*C)*e

% Simulate for each observer speed
A_err_slow   = A - L_slow*C;
A_err_medium = A - L_medium*C;
A_err_fast   = A - L_fast*C;

% Initial estimation error
e0 = x0_obs_wrong - x0_true;    % [0; 5] error in current

% Simulate error dynamics (how fast error goes to zero)
sys_err_slow   = ss(A_err_slow,   zeros(2,1), eye(2), zeros(2,1));
sys_err_medium = ss(A_err_medium, zeros(2,1), eye(2), zeros(2,1));
sys_err_fast   = ss(A_err_fast,   zeros(2,1), eye(2), zeros(2,1));

% Initial condition response (no input, just IC decay)
[~, ~, x_err_slow]   = initial(sys_err_slow,   e0, t);
[~, ~, x_err_medium] = initial(sys_err_medium, e0, t);
[~, ~, x_err_fast]   = initial(sys_err_fast,   e0, t);

%% ================================================
%  STEP 5 — PLOT ESTIMATION ERROR CONVERGENCE
%% ================================================
figure('Name','Observer Estimation Error','NumberTitle','off')

% Angular velocity estimation error (state 1)
subplot(2,1,1)
plot(t, x_err_slow(:,1),   'b-',  'LineWidth', 1.5); hold on
plot(t, x_err_medium(:,1), 'g-',  'LineWidth', 1.5)
plot(t, x_err_fast(:,1),   'r-',  'LineWidth', 1.5)
yline(0, 'k--', 'LineWidth', 1)
legend('Slow observer','Medium observer','Fast observer')
title('Angular Velocity (\omega) Estimation Error')
xlabel('Time (s)'); ylabel('Error (rad/s)')
grid on

% Armature current estimation error (state 2)
subplot(2,1,2)
plot(t, x_err_slow(:,2),   'b-',  'LineWidth', 1.5); hold on
plot(t, x_err_medium(:,2), 'g-',  'LineWidth', 1.5)
plot(t, x_err_fast(:,2),   'r-',  'LineWidth', 1.5)
yline(0, 'k--', 'LineWidth', 1)
legend('Slow observer','Medium observer','Fast observer')
title('Armature Current (i) Estimation Error')
xlabel('Time (s)'); ylabel('Error (A)')
grid on

sgtitle('Observer Convergence — Starting from Wrong Initial Estimate')

%% ================================================
%  STEP 6 — PRINT CONVERGENCE TIMES
%% ================================================
threshold = 0.01;    % consider converged when error < 1%

fprintf('=== Observer Convergence Times (to < 1%% of initial error) ===\n')
observers = {x_err_slow, x_err_medium, x_err_fast};
obs_names = {'Slow  ', 'Medium', 'Fast  '};

for k = 1:3
    err_norm = vecnorm(observers{k}, 2, 2);
    conv_idx = find(err_norm < threshold * norm(e0), 1);
    if ~isempty(conv_idx)
        fprintf('%s observer converges at t = %.4f s\n',...
            obs_names{k}, t(conv_idx))
    else
        fprintf('%s observer: did not converge in simulation window\n',...
            obs_names{k})
    end
end

%% ================================================
%  STEP 7 — PRINT OBSERVER POLES
%% ================================================
fprintf('\n=== Observer Closed-Loop Poles ===\n')
fprintf('Slow   poles: %s\n', mat2str(round(eig(A - L_slow*C)',   2)))
fprintf('Medium poles: %s\n', mat2str(round(eig(A - L_medium*C)', 2)))
fprintf('Fast   poles: %s\n', mat2str(round(eig(A - L_fast*C)',   2)))