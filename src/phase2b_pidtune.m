%% ================================================
%  PHASE 2B — Automatic PID Tuning with pidtune
%  Purva Ghanate | Servo Control Portfolio Project
%% ================================================
clear; clc; close all;

%% --- Plant (same as before) ---
R=1.0; L=0.5; K=0.01; J=0.01; b=0.1;
num_tf = [K];
den_tf = [J*L, (J*R+b*L), (b*R+K^2)];
G = tf(num_tf, den_tf);

%% --- Manual PID gains from Phase 2A (reference) ---
C_manual = pid(100, 200, 5);
CL_manual = feedback(C_manual*G, 1);

%% ================================================
%  AUTOMATIC TUNING — 3 different bandwidths
%% ================================================

% Bandwidth = how aggressively fast you want the controller
% Units: rad/s — higher = faster response but more overshoot risk

wc_slow   = 5;    % Conservative — smooth, minimal overshoot
wc_medium = 15;   % Balanced — good speed and stability
wc_fast   = 40;   % Aggressive — very fast, more overshoot

%% --- Design 3 controllers ---
opts_slow   = pidtuneOptions('CrossoverFrequency', wc_slow,   'PhaseMargin', 60);
opts_medium = pidtuneOptions('CrossoverFrequency', wc_medium, 'PhaseMargin', 60);
opts_fast   = pidtuneOptions('CrossoverFrequency', wc_fast,   'PhaseMargin', 60);

C_slow   = pidtune(G, 'PID', opts_slow);
C_medium = pidtune(G, 'PID', opts_medium);
C_fast   = pidtune(G, 'PID', opts_fast);

%% --- Print the gains ---
fprintf('=== AUTO-TUNED PID GAINS ===\n\n')

fprintf('-- Slow (wc = %d rad/s) --\n', wc_slow)
fprintf('Kp=%.4f | Ki=%.4f | Kd=%.4f\n\n',...
    C_slow.Kp, C_slow.Ki, C_slow.Kd)

fprintf('-- Medium (wc = %d rad/s) --\n', wc_medium)
fprintf('Kp=%.4f | Ki=%.4f | Kd=%.4f\n\n',...
    C_medium.Kp, C_medium.Ki, C_medium.Kd)

fprintf('-- Fast (wc = %d rad/s) --\n', wc_fast)
fprintf('Kp=%.4f | Ki=%.4f | Kd=%.4f\n\n',...
    C_fast.Kp, C_fast.Ki, C_fast.Kd)

fprintf('-- Manual (Phase 2A reference) --\n')
fprintf('Kp=%.4f | Ki=%.4f | Kd=%.4f\n\n',...
    C_manual.Kp, C_manual.Ki, C_manual.Kd)

%% --- Build closed-loop systems ---
CL_slow   = feedback(C_slow*G,   1);
CL_medium = feedback(C_medium*G, 1);
CL_fast   = feedback(C_fast*G,   1);

%% --- Plot all 4 together ---
figure('Name','pidtune Comparison','NumberTitle','off')
step(CL_slow,   'b-')
hold on
step(CL_medium, 'g-')
step(CL_fast,   'r-')
step(CL_manual, 'm--')
yline(1.0, 'k--', 'Target', 'LineWidth', 1.2)
legend('Slow (wc=5)','Medium (wc=15)','Fast (wc=40)',...
       'Manual (Phase 2A)', 'Target')
title('pidtune: Bandwidth Comparison')
xlabel('Time (s)'); ylabel('\omega (rad/s)')
grid on

%% --- Print metrics for all 4 ---
controllers = {CL_slow, CL_medium, CL_fast, CL_manual};
names = {'Slow  ', 'Medium', 'Fast  ', 'Manual'};

fprintf('=== PERFORMANCE COMPARISON ===\n')
fprintf('%-10s | %-10s | %-12s | %-10s | %-8s\n',...
    'Controller','Rise(s)','Settling(s)','Overshoot','SS Error')
fprintf('%s\n', repmat('-',1,65))

for i = 1:4
    S = stepinfo(controllers{i});
    dc = dcgain(controllers{i});
    sse = abs(1-dc)*100;
    fprintf('%-10s | %-10.4f | %-12.4f | %-10.2f | %-8.2f\n',...
        names{i}, S.RiseTime, S.SettlingTime, S.Overshoot, sse)
end

%% --- Stability check for all ---
fprintf('\n=== STABILITY CHECK ===\n')
for i = 1:3
    CL_list = {CL_slow, CL_medium, CL_fast};
    p = pole(CL_list{i});
    if all(real(p) < 0)
        fprintf('%s bandwidth: STABLE ✓\n', names{i})
    else
        fprintf('%s bandwidth: UNSTABLE ✗\n', names{i})
    end
end

%% --- Bode plot comparison ---
figure('Name','Open-Loop Bode Comparison','NumberTitle','off')
margin(C_slow*G)
hold on
margin(C_medium*G)
margin(C_fast*G)
legend('Slow','Medium','Fast')
title('Bode Plot — Phase and Gain Margins')
grid on