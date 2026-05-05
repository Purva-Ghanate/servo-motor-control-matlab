%% ================================================
%  PHASE 3A — LQR Full State Feedback Design
%  Purva Ghanate | Servo Control Portfolio Project
%% ================================================
clear; clc; close all;

%% --- Plant Parameters ---
R_m = 1.0;   % motor resistance (using R_m to avoid clash with LQR R)
L   = 0.5;
K   = 0.01;
J   = 0.01;
b   = 0.1;

%% --- State Space Matrices (from Phase 1) ---
A = [-b/J,   K/J;
     -K/L,  -R_m/L];

B = [0;
     1/L];

C = [1, 0];   % measure only omega (x1)
D = 0;

sys = ss(A, B, C, D);

%% --- Check Controllability ---
% System must be controllable for LQR to work
Co = ctrb(A, B);
rank_Co = rank(Co);
fprintf('=== Controllability Check ===\n')
fprintf('Controllability matrix rank: %d (need %d)\n', rank_Co, size(A,1))
if rank_Co == size(A,1)
    fprintf('>> System is CONTROLLABLE ✓\n\n')
else
    fprintf('>> System NOT controllable — LQR cannot be applied!\n\n')
end

%% ================================================
%  LQR DESIGN — 3 different Q/R weightings
%% ================================================

% --- Weighting 1: Conservative ---
Q1 = diag([10, 1]);     % low velocity penalty
R1 = 1;                 % high input penalty
K1 = lqr(A, B, Q1, R1);
fprintf('=== LQR Gains — Conservative ===\n')
fprintf('K = [%.4f, %.4f]\n\n', K1(1), K1(2))

% --- Weighting 2: Balanced (default) ---
Q2 = diag([100, 1]);    % medium velocity penalty
R2 = 0.01;
K2 = lqr(A, B, Q2, R2);
fprintf('=== LQR Gains — Balanced ===\n')
fprintf('K = [%.4f, %.4f]\n\n', K2(1), K2(2))

% --- Weighting 3: Aggressive ---
Q3 = diag([1000, 1]);   % high velocity penalty
R3 = 0.001;
K3 = lqr(A, B, Q3, R3);
fprintf('=== LQR Gains — Aggressive ===\n')
fprintf('K = [%.4f, %.4f]\n\n', K3(1), K3(2))

%% ================================================
%  BUILD CLOSED-LOOP SYSTEMS
%% ================================================
% With state feedback u = -Kx + N*r
% N is a feedforward gain to ensure steady-state = 1
% N = -1 / (C*(A-B*K)^-1 * B)

% Function to compute feedforward gain
get_N = @(K_gains) -1 / (C * ((A - B*K_gains) \ B));

N1 = get_N(K1);
N2 = get_N(K2);
N3 = get_N(K3);

fprintf('=== Feedforward Gains ===\n')
fprintf('N1=%.4f | N2=%.4f | N3=%.4f\n\n', N1, N2, N3)

% Closed-loop A matrices
Acl1 = A - B*K1;
Acl2 = A - B*K2;
Acl3 = A - B*K3;

% Closed-loop state space systems
sys_cl1 = ss(Acl1, B*N1, C, D);
sys_cl2 = ss(Acl2, B*N2, C, D);
sys_cl3 = ss(Acl3, B*N3, C, D);

%% --- Print Closed-Loop Poles ---
fprintf('=== Closed-Loop Poles ===\n')
fprintf('Conservative: '); disp(eig(Acl1)')
fprintf('Balanced:     '); disp(eig(Acl2)')
fprintf('Aggressive:   '); disp(eig(Acl3)')

%% ================================================
%  STEP RESPONSE COMPARISON
%% ================================================

% Also load PID from Phase 2A for comparison
G = tf([K], [J*L, (J*R_m+b*L), (b*R_m+K^2)]);
C_pid = pid(100, 200, 5);
CL_pid = feedback(C_pid*G, 1);

figure('Name','LQR vs PID Comparison','NumberTitle','off')
step(sys_cl1, 'b-')
hold on
step(sys_cl2, 'g-')
step(sys_cl3, 'r-')
step(CL_pid,  'm--')
yline(1.0, 'k--', 'Target', 'LineWidth', 1.2)
legend('LQR Conservative','LQR Balanced','LQR Aggressive','PID (Phase 2A)')
title('LQR State Feedback vs PID — Step Response')
xlabel('Time (s)'); ylabel('\omega (rad/s)')
grid on

%% --- Performance Metrics Table ---
systems  = {sys_cl1, sys_cl2, sys_cl3, CL_pid};
names    = {'Conservative', 'Balanced    ', 'Aggressive  ', 'PID (2A)    '};

fprintf('=== PERFORMANCE COMPARISON ===\n')
fprintf('%-14s | %-8s | %-11s | %-10s | %-8s\n',...
    'Controller','Rise(s)','Settling(s)','Overshoot','SS Error')
fprintf('%s\n', repmat('-',1,62))

for i = 1:4
    S   = stepinfo(systems{i});
    dc  = dcgain(systems{i});
    sse = abs(1-dc)*100;
    fprintf('%-14s | %-8.4f | %-11.4f | %-10.2f | %-8.2f\n',...
        names{i}, S.RiseTime, S.SettlingTime, S.Overshoot, sse)
end