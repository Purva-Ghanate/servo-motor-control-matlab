%% ================================================
%  PHASE 2A — Manual PID Tuning
%  Purva Ghanate | Servo Control Portfolio Project
%% ================================================
clear; clc; close all;

%% --- Motor Parameters (same as Phase 1) ---
R  = 1.0;
L  = 0.5;
K  = 0.01;
J  = 0.01;
b  = 0.1;

%% --- Plant Transfer Function (same as Phase 1) ---
num_tf = [K];
den_tf = [J*L, (J*R + b*L), (b*R + K^2)];
G = tf(num_tf, den_tf);

%% --- Define PID Gains (YOU will tune these) ---
Kp = 100;    % Proportional gain
Ki = 200;    % Integral gain
Kd = 5;      % Derivative gain

%% --- Build PID Controller ---
C = pid(Kp, Ki, Kd);
fprintf('=== PID Controller ===\n')
C

%% --- Build Closed-Loop System ---
% feedback(C*G, 1) means unity feedback
CL = feedback(C*G, 1);
fprintf('=== Closed-Loop Transfer Function ===\n')
CL

%% --- Step Response of Closed-Loop ---
figure('Name','Manual PID Step Response','NumberTitle','off')
step(CL)
title(sprintf('Closed-Loop Step Response | Kp=%.0f Ki=%.0f Kd=%.0f',...
               Kp, Ki, Kd))
xlabel('Time (s)'); ylabel('\omega (rad/s)')
yline(1.0, 'r--', 'Target', 'LineWidth', 1.5)
grid on

%% --- Print Performance Metrics ---
S = stepinfo(CL);
fprintf('\n=== Performance Metrics ===\n')
fprintf('Rise Time:     %.4f s\n', S.RiseTime)
fprintf('Settling Time: %.4f s\n', S.SettlingTime)
fprintf('Overshoot:     %.2f %%\n', S.Overshoot)
fprintf('Peak:          %.4f\n',   S.Peak)

% Steady-state value
dc_gain = dcgain(CL);
ss_error = abs(1 - dc_gain) * 100;
fprintf('DC Gain:       %.4f\n',   dc_gain)
fprintf('Steady-State Error: %.2f %%\n', ss_error)

%% --- Check Stability ---
CL_poles = pole(CL);
fprintf('\n=== Closed-Loop Poles ===\n')
disp(CL_poles)
if all(real(CL_poles) < 0)
    fprintf('>> Closed-Loop is STABLE ✓\n')
else
    fprintf('>> UNSTABLE — retune gains!\n')
end

%% --- Compare Open-Loop vs Closed-Loop ---
figure('Name','Open vs Closed Loop','NumberTitle','off')
step(G, 'b--')      % open loop
hold on
step(CL, 'r-')      % closed loop with PID
yline(1.0, 'k--', 'Target', 'LineWidth', 1.5)
legend('Open-Loop (no controller)','PID Closed-Loop','Target')
title('Open-Loop vs PID Closed-Loop')
xlabel('Time (s)'); ylabel('\omega (rad/s)')
grid on