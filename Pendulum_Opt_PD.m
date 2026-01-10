% =========================================================================
% OPTIMIZACIÓN DEL PÉNDULO INVERTIDO - SOLO PD
% =========================================================================
% Estilo helicóptero: K_angle fijo, Differential Evolution optimiza solo PD
%
% IMPORTANTE: Antes de ejecutar, fijar K_angle en el modelo:
%   K_angle_fijo = 5000;
%   set_param('rct_pendulum/Angle Controller', 'Gain', num2str(K_angle_fijo));
%   save_system('rct_pendulum');
% =========================================================================

clear all;
close all;
clc;

% -------------------------------------------------------------------------
% CONFIGURACIÓN
% -------------------------------------------------------------------------
K_angle_fijo = 1000;  % Valor fijo (calculado previamente)

fprintf('=========================================================================\n');
fprintf('OPTIMIZACIÓN PÉNDULO INVERTIDO - SOLO PD (K_angle fijo)\n');
fprintf('=========================================================================\n');
fprintf('Similar al helicóptero: SOF fijo, PID optimizado con DE\n');
fprintf('K_angle (fijo): %.0f\n', K_angle_fijo);
fprintf('Parámetros a optimizar: 2 (Kp_pos, Kd_pos)\n');
fprintf('=========================================================================\n\n');

% -------------------------------------------------------------------------
% OPTIMIZATION PARAMETERS
% -------------------------------------------------------------------------
VTR = 1.e-6;            % Value to reach (stopping criterion)
D = 2;                  % SOLO 2 parámetros: Kp y Kd

% Parameter bounds
XVmin = [0, 0];         % [Kp_min, Kd_min]
XVmax = [20, 15];       % [Kp_max, Kd_max]

y = [0, 0];             % Additional data (not used)

% DE algorithm parameters
NP = 60;                % Population size (más pequeña: solo 2 params)
itermax = 40;           % Maximum iterations

F = 0.7;                % Differential weight (como helicóptero)
CR = 0.5;               % Crossover probability (como helicóptero)

strategy = 7;           % DE/rand/1/bin strategy
refresh = 5;            % Display progress every N iterations

fprintf('Population size: %d\n', NP);
fprintf('Max iterations: %d\n', itermax);
fprintf('Kp range: [%.1f, %.1f]\n', XVmin(1), XVmax(1));
fprintf('Kd range: [%.1f, %.1f]\n', XVmin(2), XVmax(2));
fprintf('=========================================================================\n\n');

% -------------------------------------------------------------------------
% INITIALIZATION
% -------------------------------------------------------------------------
load_system('rct_pendulum');
set_param('rct_pendulum', 'SimMechanicsOpenEditorOnUpdate', 'off');
save_system('rct_pendulum');
% Verificar que K_angle está fijo
try
    gain_value = get_param('rct_pendulum/Angle Controller', 'Gain');
    fprintf('✅ K_angle en modelo: %s\n', gain_value);
    if ~strcmp(gain_value, num2str(K_angle_fijo))
        warning('K_angle en modelo (%s) no coincide con configuración (%.0f)', ...
                gain_value, K_angle_fijo);
        fprintf('Configurando K_angle...\n');
        set_param('rct_pendulum/Angle Controller', 'Gain', num2str(K_angle_fijo));
        save_system('rct_pendulum');
        fprintf('✅ K_angle actualizado\n');
    end
catch
    warning('No se pudo verificar K_angle. Asegúrate de que esté fijo en %.0f', K_angle_fijo);
end

fprintf('\n🚀 Iniciando optimización...\n\n');

% -------------------------------------------------------------------------
% RUN OPTIMIZATION
% -------------------------------------------------------------------------
tic;
[x, f, nf] = devec3('pendulum_cost_PD', VTR, D, XVmin, XVmax, y, NP, itermax, F, CR, strategy, refresh);
tiempo_total = toc;

% -------------------------------------------------------------------------
% RESULTS
% -------------------------------------------------------------------------
fprintf('\n=========================================================================\n');
fprintf('OPTIMIZACIÓN COMPLETADA ✅\n');
fprintf('=========================================================================\n');
fprintf('Total time: %.1f seconds (%.1f minutes)\n', tiempo_total, tiempo_total/60);
fprintf('Function evaluations: %d\n', nf);
fprintf('Best cost: %.4e\n\n', f);

fprintf('PARÁMETROS ÓPTIMOS DEL CONTROLADOR PD:\n');
Kp_opt = x(1);
Kd_opt = x(2);

fprintf('  Kp_pos = %.6f\n', Kp_opt);
fprintf('  Kd_pos = %.6f\n', Kd_opt);
fprintf('  K_angle = %.0f (fijo)\n', K_angle_fijo);
fprintf('=========================================================================\n\n');

% -------------------------------------------------------------------------
% SIMULATE WITH OPTIMAL PARAMETERS
% -------------------------------------------------------------------------
fprintf('Simulando con parámetros óptimos...\n');

assignin('base', 'Kp_pos', Kp_opt);
assignin('base', 'Kd_pos', Kd_opt);

simOut = sim('rct_pendulum', 'StopTime', '12');

% -------------------------------------------------------------------------
% GENERATE PLOTS
% -------------------------------------------------------------------------
fprintf('Generando gráficas...\n');

figure('Position', [100 100 1400 800]);

% Subplot 1: Position tracking
subplot(2,3,1);
plot(simOut.tout, simOut.xref, 'b--', simOut.tout, simOut.x, 'r-', 'LineWidth', 2);
legend('Referencia', 'Posición');
xlabel('Tiempo (s)');
ylabel('Posición (m)');
title('Seguimiento de Posición del Carro');
grid on;

% Subplot 2: Pendulum angle
subplot(2,3,2);
plot(simOut.tout, simOut.Theta*180/pi, 'g-', 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Ángulo (grados)');
title('Ángulo del Péndulo');
yline(0, 'k--', 'Vertical');
grid on;

% Subplot 3: Control force
subplot(2,3,3);
plot(simOut.tout, simOut.F, 'm-', 'LineWidth', 2);
xlabel('Tiempo (s)');
ylabel('Fuerza (N)');
title('Señal de Control');
grid on;

% Subplot 4: Position error
subplot(2,3,4);
error_pos = simOut.x - simOut.xref;
plot(simOut.tout, error_pos, 'k-', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Error (m)');
title('Error de Posición');
yline(0, 'k--');
grid on;

% Subplot 5: Phase portrait
subplot(2,3,5);
plot(simOut.Theta*180/pi, gradient(simOut.Theta)*180/pi, 'b-', 'LineWidth', 1.5);
xlabel('Ángulo (grados)');
ylabel('Velocidad Angular (grados/s)');
title('Retrato de Fase del Péndulo');
grid on;

% Subplot 6: Results summary
subplot(2,3,6);
axis off;
text(0.1, 0.95, '📊 RESULTADOS FINALES', 'FontSize', 13, 'FontWeight', 'bold');
text(0.1, 0.82, 'Parámetros Óptimos:', 'FontSize', 11, 'FontWeight', 'bold');
text(0.15, 0.72, sprintf('Kp = %.4f', Kp_opt), 'FontSize', 10);
text(0.15, 0.62, sprintf('Kd = %.4f', Kd_opt), 'FontSize', 10);
text(0.15, 0.52, sprintf('K_{angle} = %.0f (fijo)', K_angle_fijo), 'FontSize', 10);

text(0.1, 0.38, 'Desempeño:', 'FontSize', 11, 'FontWeight', 'bold');
text(0.15, 0.28, sprintf('Coste: %.2f', f), 'FontSize', 10);
text(0.15, 0.18, sprintf('Evaluaciones: %d', nf), 'FontSize', 10);
text(0.15, 0.08, sprintf('Tiempo: %.1f min', tiempo_total/60), 'FontSize', 10);

sgtitle('Optimización del Péndulo Invertido con Differential Evolution (Solo PD)', ...
        'FontSize', 14, 'FontWeight', 'bold');

% -------------------------------------------------------------------------
% SAVE RESULTS
% -------------------------------------------------------------------------
fprintf('Guardando resultados...\n');

results = struct();
results.Kp_optimal = Kp_opt;
results.Kd_optimal = Kd_opt;
results.K_angle_fixed = K_angle_fijo;
results.best_cost = f;
results.num_evaluations = nf;
results.computation_time = tiempo_total;
results.DE_parameters = struct('NP', NP, 'itermax', itermax, 'F', F, 'CR', CR, 'strategy', strategy);
results.simulation_data = struct('time', simOut.tout, 'position', simOut.x, ...
                                 'reference', simOut.xref, 'angle', simOut.Theta, 'force', simOut.F);

save('pendulum_PD_optimization_results.mat', 'results');

fprintf('\n✅ ¡Optimización finalizada con éxito!\n');
fprintf('=========================================================================\n');
