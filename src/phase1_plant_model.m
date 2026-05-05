%% ================================================
%  PHASE 1 — PMDC Motor Plant Modeling
%  Purva Ghanate | Servo Control Portfolio Project
%% ================================================
clear; clc; close all;

%% --- Step 1: Define Motor Parameters ---
% Realistic PMDC motor values (standard lab motor)
R  = 1.0;    % Armature resistance [Ohm]
L  = 0.5;    % Armature inductance [H]
K  = 0.01;   % Motor constant (Ke = Kt = K) [V.s/rad or N.m/A]
J  = 0.01;   % Rotor moment of inertia [kg.m^2]
b  = 0.1;    % Viscous friction coefficient [N.m.s]

fprintf('=== Motor Parameters ===\n')
fprintf('R=%.2f Ω | L=%.2f H | K=%.4f | J=%.4f kg.m² | b=%.2f\n',...
         R, L, K, J, b)

%% --- Step 2: Transfer Function G(s) = Omega(s)/V(s) ---
% Numerator: K
% Denominator: (Js+b)(Ls+R) + K^2
%   = JL.s^2 + (JR + bL).s + (bR + K^2)

num_tf = [K];
den_tf = [J*L, (J*R + b*L), (b*R + K^2)];

G = tf(num_tf, den_tf);

fprintf('\n=== Transfer Function G(s) ===\n')
G

%% --- Step 3: State-Space Model ---
A = [-b/J,  K/J;
     -K/L, -R/L];

B = [0;
     1/L];

C = [1, 0];   % Output: angular velocity (x1)

D = 0;

sys = ss(A, B, C, D);

fprintf('=== State-Space Matrices ===\n')
fprintf('A matrix:\n'); disp(A)
fprintf('B matrix:\n'); disp(B)
fprintf('C matrix:\n'); disp(C)

%% --- Step 4: Verify TF matches SS ---
G_from_ss = tf(sys);
fprintf('=== TF from SS (should match G above) ===\n')
G_from_ss

%% --- Step 5: Pole-Zero Analysis ---
fprintf('=== Poles of G(s) ===\n')
disp(pole(G))

fprintf('=== Zeros of G(s) ===\n')
disp(zero(G))

% Are poles in left-half plane? (stable if yes)
if all(real(pole(G)) < 0)
    fprintf('>> Plant is OPEN-LOOP STABLE ✓\n')
else
    fprintf('>> Plant is OPEN-LOOP UNSTABLE — controller needed!\n')
end
%% --- Step 6: Open-Loop Step Response ---
figure('Name','Step Response','NumberTitle','off')
step(G)
title('Open-Loop Step Response — \omega(t) to unit voltage step')
xlabel('Time (s)'); ylabel('\omega (rad/s)')
grid on

% Extract step info
S = stepinfo(G);
fprintf('\n=== Step Response Metrics ===\n')
fprintf('Rise Time:     %.4f s\n', S.RiseTime)
fprintf('Settling Time: %.4f s\n', S.SettlingTime)
fprintf('Overshoot:     %.2f %%\n', S.Overshoot)
fprintf('Peak:          %.4f\n',   S.Peak)

%% --- Step 7: Pole-Zero Map ---
figure('Name','Pole-Zero Map','NumberTitle','off')
pzmap(G)
title('Pole-Zero Map — Open Loop G(s)')
grid on

%% --- Step 8: Impulse Response ---
figure('Name','Impulse Response','NumberTitle','off')
impulse(G)
title('Open-Loop Impulse Response')
xlabel('Time (s)'); ylabel('\omega (rad/s)')
grid on

%% --- Step 9: Bode Plot ---
figure('Name','Bode Plot','NumberTitle','off')
bode(G)
title('Open-Loop Bode Plot')
grid on

%% --- Step 10: Print Gain & Phase Margins ---
[Gm, Pm, Wcg, Wcp] = margin(G);
fprintf('\n=== Stability Margins ===\n')
fprintf('Gain Margin:  %.2f dB (at %.2f rad/s)\n', 20*log10(Gm), Wcg)
fprintf('Phase Margin: %.2f deg (at %.2f rad/s)\n', Pm, Wcp)