% =========================================================================
% INVERTED PENDULUM OPTIMIZATION USING DIFFERENTIAL EVOLUTION
% =========================================================================
% Optimizes 3 parameters: Kp_pos, Kd_pos, K_angle
% Uses the DE algorithm (devec3) to find optimal controller gains

clear;
clc;

% -------------------------------------------------------------------------
% OPTIMIZATION PARAMETERS
% -------------------------------------------------------------------------
VTR = 1.e-6;            % Value to reach (stopping criterion)
D = 3;                  % Number of parameters to optimize

% Parameter bounds
XVmin = [0, 0, 0];      % [Kp_min, Kd_min, K_angle_min]
XVmax = [15, 8, 100];   % [Kp_max, Kd_max, K_angle_max]

y = [0, 0, 0];          % Additional data (not used)

% DE algorithm parameters
NP = 50;                % Population size (reduce to 30 for faster testing)
itermax = 30;           % Maximum iterations (reduce to 20 for testing)
F = 0.8;                % Differential weight [0.5, 1.0]
CR = 0.9;               % Crossover probability [0, 1]
strategy = 7;           % DE/rand/1/bin strategy
refresh = 5;            % Display progress every N iterations

% -------------------------------------------------------------------------
% INITIALIZATION
% -------------------------------------------------------------------------
load_system('rct_pendulum');

fprintf('=======================================================\n');
fprintf('INVERTED PENDULUM OPTIMIZATION\n');
fprintf('=======================================================\n');
fprintf('Parameters to optimize: 3 (Kp_pos, Kd_pos, K_angle)\n');
fprintf('Population size: %d\n', NP);
fprintf('Max iterations: %d\n', itermax);
fprintf('Kp range: [%.1f, %.1f]\n', XVmin(1), XVmax(1));
fprintf('Kd range: [%.1f, %.1f]\n', XVmin(2), XVmax(2));
fprintf('K_angle range: [%.1f, %.1f]\n', XVmin(3), XVmax(3));
fprintf('=======================================================\n\n');

% -------------------------------------------------------------------------
% RUN OPTIMIZATION
% -------------------------------------------------------------------------
tic;
[x, f, nf] = devec3('pendulum_cost', VTR, D, XVmin, XVmax, y, NP, itermax, F, CR, strategy, refresh);
tiempo_total = toc;

% -------------------------------------------------------------------------
% DISPLAY RESULTS
% -------------------------------------------------------------------------
fprintf('\n=======================================================\n');
fprintf('OPTIMIZATION COMPLETED\n');
fprintf('=======================================================\n');
fprintf('Total time: %.1f seconds (%.1f minutes)\n', tiempo_total, tiempo_total/60);
fprintf('Function evaluations: %d\n', nf);
fprintf('Best cost: %.4e\n\n', f);

Kp = x(1);
Kd = x(2);
K_angle = x(3);

fprintf('OPTIMAL PARAMETERS:\n');
fprintf('  Kp_pos    = %.4f\n', Kp);
fprintf('  Kd_pos    = %.4f\n', Kd);
fprintf('  K_angle   = %.4f\n', K_angle);
fprintf('=======================================================\n\n');

% -------------------------------------------------------------------------
% VERIFICATION SIMULATION
% -------------------------------------------------------------------------
assignin('base', 'Kp_pos', Kp);
assignin('base', 'Kd_pos', Kd);
assignin('base', 'K_angle', K_angle);

fprintf('Running verification simulation...\n');
simOut = sim('rct_pendulum', 'StopTime', '12');

% -------------------------------------------------------------------------
% GENERATE PLOTS
% -------------------------------------------------------------------------
fprintf('Generating plots...\n');
figure('Name', 'Optimization Results', 'NumberTitle', 'off');

% Cart position
subplot(2,2,1);
plot(simOut.tout, simOut.xref, 'b--', simOut.tout, simOut.x, 'r-', 'LineWidth', 1.5);
legend('Reference', 'Position', 'Location', 'best');
xlabel('Time (s)');
ylabel('Position (m)');
title('Cart Position');
grid on;

% Pendulum angle
subplot(2,2,2);
plot(simOut.tout, simOut.Theta * 180/pi, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Pendulum Angle');
yline(0, 'k--', 'Vertical');
grid on;

% Control force
subplot(2,2,3);
plot(simOut.tout, simOut.F, 'm-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Force (N)');
title('Control Force');
grid on;

% Results summary
subplot(2,2,4);
axis off;
text(0.1, 0.9, 'RESULTS SUMMARY', 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.75, sprintf('Final cost: %.4e', f), 'FontSize', 10);
text(0.1, 0.65, sprintf('Kp_{pos}: %.4f', Kp), 'FontSize', 10);
text(0.1, 0.55, sprintf('Kd_{pos}: %.4f', Kd), 'FontSize', 10);
text(0.1, 0.45, sprintf('K_{angle}: %.4f', K_angle), 'FontSize', 10);
text(0.1, 0.35, sprintf('Evaluations: %d', nf), 'FontSize', 10);
text(0.1, 0.25, sprintf('Time: %.1f min', tiempo_total/60), 'FontSize', 10);
text(0.1, 0.10, '[OK] Optimization completed', 'FontSize', 10, 'Color', [0 0.6 0]);

fprintf('\n[OK] Optimization finished successfully!\n');
fprintf('=======================================================\n');