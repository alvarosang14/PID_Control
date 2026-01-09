% Script de Optimización del Péndulo Invertido con Evolución Diferencial
% Adaptado del código original de PID con DE

clear all;
close all;
clc;

%% Parámetros de Evolución Diferencial
VTR = 1.e-6;          % Valor objetivo a alcanzar (muy pequeño = óptimo)
D = 4;                 % Número de parámetros a optimizar

% Límites de búsqueda para cada parámetro
% [Kp_pos, Kd_pos, K1_angle, K2_angle]
XVmin = [0,    0,    0,     0];      % Límites inferiores
XVmax = [15,   8,    80,    80];     % Límites superiores

y = [0, 0, 0, 0];     % Vector de datos del problema (no usado aquí)

NP = 60;              % Tamaño de la población (más grande = más exploración)
itermax = 30;         % Número máximo de iteraciones
F = 0.8;              % Factor de escala DE (diferencial)
CR = 0.9;             % Probabilidad de cruce
strategy = 7;         % Estrategia: 7 = DE/rand/1/bin (buena para robustez)
refresh = 5;          % Mostrar progreso cada N iteraciones

%% Cargar o inicializar el modelo de Simulink
modelo = 'rct_pendulum';

% Verificar que el modelo existe
if exist([modelo '.slx'], 'file') ~= 4
    error('El modelo %s.slx no se encuentra. Asegúrate de tenerlo en el path.', modelo);
end

% Cargar el modelo sin abrirlo
load_system(modelo);

fprintf('=================================================\n');
fprintf('OPTIMIZACIÓN DEL PÉNDULO INVERTIDO CON DE\n');
fprintf('=================================================\n');
fprintf('Población: %d individuos\n', NP);
fprintf('Iteraciones máximas: %d\n', itermax);
fprintf('Parámetros a optimizar: %d\n', D);
fprintf('Estrategia DE: %d\n', strategy);
fprintf('=================================================\n\n');

%% Ejecutar Evolución Diferencial
tic;  % Iniciar cronómetro

[x_opt, f_opt, nfeval] = devec3('pendulum_cost', VTR, D, XVmin, XVmax, y, ...
                                 NP, itermax, F, CR, strategy, refresh);

tiempo_total = toc;  % Detener cronómetro

%% Mostrar Resultados
fprintf('\n=================================================\n');
fprintf('OPTIMIZACIÓN COMPLETADA\n');
fprintf('=================================================\n');
fprintf('Tiempo total: %.2f segundos\n', tiempo_total);
fprintf('Evaluaciones de función: %d\n', nfeval);
fprintf('Mejor coste alcanzado: %e\n', f_opt);
fprintf('\nParámetros óptimos encontrados:\n');
fprintf('  Kp_posicion = %.4f\n', x_opt(1));
fprintf('  Kd_posicion = %.4f\n', x_opt(2));
fprintf('  K1_angulo   = %.4f\n', x_opt(3));
fprintf('  K2_angulo   = %.4f\n', x_opt(4));
fprintf('=================================================\n\n');

%% Aplicar parámetros óptimos al modelo
fprintf('Aplicando parámetros óptimos al modelo...\n');

Kp_pos = x_opt(1);
Kd_pos = x_opt(2);
K1_angle = x_opt(3);
K2_angle = x_opt(4);

% Configurar el modelo con los valores óptimos
% NOTA: Ajusta estos comandos según la estructura real de tu modelo
try
    set_param([modelo '/Position Controller'], 'P', num2str(Kp_pos));
    set_param([modelo '/Position Controller'], 'D', num2str(Kd_pos));
    set_param([modelo '/Angle Controller'], 'Gain1', num2str(K1_angle));
    set_param([modelo '/Angle Controller'], 'Gain2', num2str(K2_angle));
    
    fprintf('Parámetros aplicados correctamente.\n');
catch ME
    warning('No se pudieron aplicar automáticamente los parámetros: %s', ME.message);
    fprintf('Aplica manualmente estos valores en el modelo.\n');
end

%% Simulación final con parámetros óptimos
fprintf('\nEjecutando simulación final...\n');

try
    [tout, xout, yout] = sim(modelo, [0 12]);
    
    % Graficar resultados
    figure('Name', 'Resultados de Optimización', 'Position', [100 100 1200 600]);
    
    % Subplot 1: Posición del carro
    subplot(2,2,1);
    plot(tout, yout(:,1), 'b-', 'LineWidth', 1.5); hold on;
    if size(yout,2) >= 2
        plot(tout, yout(:,2), 'r--', 'LineWidth', 1.5);
        legend('Referencia', 'Posición Real', 'Location', 'best');
    end
    xlabel('Tiempo (s)');
    ylabel('Posición (m)');
    title('Seguimiento de Posición del Carro');
    grid on;
    
    % Subplot 2: Ángulo del péndulo
    subplot(2,2,2);
    if size(yout,2) >= 3
        plot(tout, yout(:,3) * 180/pi, 'g-', 'LineWidth', 1.5);
        ylabel('Ángulo (grados)');
    end
    xlabel('Tiempo (s)');
    title('Ángulo del Péndulo (debe estar cerca de 0°)');
    grid on;
    
    % Subplot 3: Fuerza de control
    subplot(2,2,3);
    if size(yout,2) >= 4
        plot(tout, yout(:,4), 'm-', 'LineWidth', 1.5);
        ylabel('Fuerza (N)');
    end
    xlabel('Tiempo (s)');
    title('Esfuerzo de Control');
    grid on;
    
    % Subplot 4: Métricas de rendimiento
    subplot(2,2,4);
    text(0.1, 0.8, sprintf('Coste Final: %.4e', f_opt), 'FontSize', 12);
    text(0.1, 0.7, sprintf('Kp_{pos}: %.4f', Kp_pos), 'FontSize', 11);
    text(0.1, 0.6, sprintf('Kd_{pos}: %.4f', Kd_pos), 'FontSize', 11);
    text(0.1, 0.5, sprintf('K1_{ang}: %.4f', K1_angle), 'FontSize', 11);
    text(0.1, 0.4, sprintf('K2_{ang}: %.4f', K2_angle), 'FontSize', 11);
    text(0.1, 0.3, sprintf('Evaluaciones: %d', nfeval), 'FontSize', 11);
    text(0.1, 0.2, sprintf('Tiempo: %.2f s', tiempo_total), 'FontSize', 11);
    axis off;
    title('Parámetros Optimizados');
    
    fprintf('Simulación completada. Revisa las gráficas.\n');
    
catch ME
    warning('Error en la simulación final: %s', ME.message);
end

%% Guardar resultados
save('pendulum_optimization_results.mat', 'x_opt', 'f_opt', 'nfeval', ...
     'Kp_pos', 'Kd_pos', 'K1_angle', 'K2_angle', 'tiempo_total');

fprintf('\nResultados guardados en: pendulum_optimization_results.mat\n');
fprintf('=================================================\n');