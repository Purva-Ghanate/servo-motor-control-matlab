%% ================================================
%  PHASE 3C — LQR + Luenberger Observer Combined
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
n = size(A,1);    % number of states = 2

%% ================================================
%  STEP 1 — LQR CONTROLLER (Balanced from 3A)
%% ================================================
Q     = diag([100, 1]);
R_lqr = 0.01;
K_lqr = lqr(A, B, Q, R_lqr);
N     = -1 / (C * ((A - B*K_lqr) \ B));

fprintf('=== LQR Controller ===\n')
fprintf('K = [%.4f, %.4f] | N = %.4f\n\n', K_lqr(1), K_lqr(2), N)

%% ================================================
%  STEP 2 — OBSERVER (Medium, 5x rule from 3B)
%% ================================================
obs_poles = [-80, -85];
L_obs     = place(A', C', obs_poles)';

fprintf('=== Observer ===\n')
fprintf('Poles: [%d, %d]\n', obs_poles(1), obs_poles(2))
fprintf('L = [%.4f, %.4f]\n\n', L_obs(1), L_obs(2))

%% ================================================
%  STEP 3 — BUILD AUGMENTED SYSTEM
%  State vector: [x (real); x_hat (estimated)]
%  Size: 4x1
%================================================
% Real plant:     x_dot     = A*x + B*u
% Observer:       x_hat_dot = (A-L*C)*x_hat + B*u + L*C*x
% Control law:    u         = -K*x_hat + N*r

% Augmented A matrix (4x4):
%   [A,         -B*K_lqr          ]
%   [L_obs*C,    A-B*K_lqr-L_obs*C]

A_aug = [A,              -B*K_lqr;
         L_obs*C,   A - B*K_lqr - L_obs*C];

% Augmented B matrix (reference input r)
B_aug = [B*N;
         B*N];

% Output: only measure real omega (x1)
C_aug = [C, zeros(1,n)];
D_aug = 0;

sys_combined = ss(A_aug, B_aug, C_aug, D_aug);

%% ================================================
%  STEP 4 — FULL STATE FEEDBACK (ideal, 3A)
%  For comparison — assumes perfect state knowledge
%% ================================================
Acl_ideal = A - B*K_lqr;
sys_ideal  = ss(Acl_ideal, B*N, C, D);

%% ================================================
%  STEP 5 — PID (Phase 2A) for comparison
%% ================================================
G      = tf([K], [J*L, (J*R_m+b*L), (b*R_m+K^2)]);
C_pid  = pid(100, 200, 5);
CL_pid = feedback(C_pid*G, 1);

%% ================================================
%  STEP 6 — VERIFY SEPARATION PRINCIPLE
%% ================================================
combined_poles = eig(A_aug);
fprintf('=== Augmented System Poles (Separation Principle) ===\n')
ctrl_eig = eig(A - B*K_lqr);
fprintf('Controller poles: %.4f%+.4fi\n', ...
    real(ctrl_eig(1)), imag(ctrl_eig(1)))
fprintf('                  %.4f%+.4fi\n', ...
    real(ctrl_eig(2)), imag(ctrl_eig(2)))
fprintf('Observer poles:   %.4f, %.4f\n\n', obs_poles(1), obs_poles(2))
fprintf('Combined system poles:\n')
for i = 1:length(combined_poles)
    fprintf('  %.4f%+.4fi\n', real(combined_poles(i)), imag(combined_poles(i)))
end

%% ================================================
%  STEP 7 — STEP RESPONSE COMPARISON
%% ================================================
figure('Name','LQR+Observer vs Ideal vs PID','NumberTitle','off')
step(sys_combined, 'g-')
hold on
step(sys_ideal,    'b--')
step(CL_pid,       'm-.')
yline(1.0, 'k--', 'Target', 'LineWidth', 1.2)
legend('LQR + Observer (realistic)',...
       'LQR Full State (ideal)',...
       'PID (Phase 2A)',...
       'Target')
title('LQR+Observer vs Full State LQR vs PID')
xlabel('Time (s)'); ylabel('\omega (rad/s)')
grid on

%% ================================================
%  STEP 8 — DISTURBANCE REJECTION TEST
%  Inject a step load disturbance at t=2s
%  (simulates press force hitting the axis)
%% ================================================
% Disturbance enters at plant input (like a load torque)
% Augmented system with disturbance input
B_dist_aug = [B; zeros(n,1)];    % disturbance hits real plant only
sys_dist   = ss(A_aug, ...
    [B_aug, B_dist_aug], C_aug, [0, 0]);

% Simulate: reference step at t=0, disturbance step at t=2
t_sim  = 0:0.001:5;
r_sig  = ones(size(t_sim));               % reference = 1
d_sig  = zeros(size(t_sim));
d_sig(t_sim >= 2) = -0.5;                % disturbance: -0.5 at t=2s

u_combined = [r_sig; d_sig];

[y_dist, ~] = lsim(sys_dist, u_combined, t_sim);

% PID disturbance response
G_dist    = tf([K], [J*L, (J*R_m+b*L), (b*R_m+K^2)]);
CL_dist   = feedback(C_pid * G_dist, 1) * tf(1,1);

% For PID disturbance: sensitivity function
S_pid     = feedback(1, C_pid*G_dist);   % sensitivity
y_pid_ref = lsim(CL_pid,  r_sig, t_sim);
y_pid_dis = lsim(S_pid*G_dist, d_sig, t_sim);
y_pid_tot = y_pid_ref + y_pid_dis;

figure('Name','Disturbance Rejection','NumberTitle','off')
plot(t_sim, y_dist,    'g-',  'LineWidth', 1.5); hold on
plot(t_sim, y_pid_tot, 'm-.', 'LineWidth', 1.5)
yline(1.0,  'k--', 'Target',     'LineWidth', 1.2)
xline(2.0,  'r--', 'Disturbance hits', 'LineWidth', 1.2)
legend('LQR + Observer','PID')
title('Disturbance Rejection — Step Load at t=2s')
xlabel('Time (s)'); ylabel('\omega (rad/s)')
grid on

%% ================================================
%  STEP 9 — PERFORMANCE METRICS
%% ================================================
systems = {sys_combined, sys_ideal, CL_pid};
names   = {'LQR+Observer', 'LQR Ideal   ', 'PID         '};

fprintf('\n=== FINAL PERFORMANCE COMPARISON ===\n')
fprintf('%-14s | %-8s | %-11s | %-10s | %-8s\n',...
    'Controller','Rise(s)','Settling(s)','Overshoot','SS Error')
fprintf('%s\n', repmat('-',1,62))

for i = 1:3
    S   = stepinfo(systems{i});
    dc  = dcgain(systems{i});
    sse = abs(1-dc)*100;
    fprintf('%-14s | %-8.4f | %-11.4f | %-10.2f | %-8.2f\n',...
        names{i}, S.RiseTime, S.SettlingTime, S.Overshoot, sse)
end