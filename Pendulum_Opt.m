% Optimización del Péndulo Invertido con Evolución Diferencial
% Estilo simple - Optimiza 3 parámetros: Kp_pos, Kd_pos, K_angle

VTR = 1.e-6; 

D = 3;  % 3 parámetros ahora

XVmin = [0, 0, 0];      % [Kp_min, Kd_min, K_angle_min]
XVmax = [15, 8, 100];   % [Kp_max, Kd_max, K_angle_max]

y = [0, 0, 0]; 

NP = 50;        % Población (puedes bajar a 30 para ir más rápido)

itermax = 30;   % Iteraciones (puedes bajar a 20 para pruebas)

F = 0.8; 

CR = 0.9; 

strategy = 7;

refresh = 5; 

% Cargar modelo
load_system('rct_pendulum');

fprintf('=======================================================\n');
fprintf('OPTIMIZACIÓN DEL PÉNDULO INVERTIDO\n');
fprintf('=======================================================\n');
fprintf('Parámetros a optimizar: 3 (Kp_pos, Kd_pos, K_angle)\n');
fprintf('Población: %d individuos\n', NP);
fprintf('Iteraciones: %d\n', itermax);
fprintf('Rango Kp: [%.1f, %.1f]\n', XVmin(1), XVmax(1));
fprintf('Rango Kd: [%.1f, %.1f]\n', XVmin(2), XVmax(2));
fprintf('Rango K_angle: [%.1f, %.1f]\n', XVmin(3), XVmax(3));
fprintf('=======================================================\n\n');

% Ejecutar optimización
tic;
[x, f, nf] = devec3('pendulum_cost', VTR, D, XVmin, XVmax, y, NP, itermax, F, CR, strategy, refresh);
tiempo_total = toc;

% Resultados
fprintf('\n=======================================================\n');
fprintf('OPTIMIZACIÓN COMPLETADA ✅\n');
fprintf('=======================================================\n');
fprintf('Tiempo total: %.1f segundos (%.1f minutos)\n', tiempo_total, tiempo_total/60);
fprintf('Evaluaciones: %d\n', nf);
fprintf('Mejor coste: %.4e\n\n', f);

Kp = x(1)
Kd = x(2)
K_angle = x(3)

fprintf('\n=======================================================\n\n');

% Aplicar parámetros óptimos
assignin('base', 'Kp_pos', Kp);
assignin('base', 'Kd_pos', Kd);
assignin('base', 'K_angle', K_angle);

% Simular con parámetros óptimos
fprintf('Simulando con parámetros óptimos...\n');
simOut = sim('rct_pendulum', 'StopTime', '12');

% Graficar
fprintf('Generando gráficas...\n');
figure(1);

subplot(2,2,1);
plot(simOut.tout, simOut.xref, 'b--', simOut.tout, simOut.x, 'r-', 'LineWidth', 1.5);
legend('Referencia', 'Posición', 'Location', 'best');
xlabel('Tiempo (s)');
ylabel('Posición (m)');
title('Posición del Carro');
grid on;

subplot(2,2,2);
plot(simOut.tout, simOut.Theta * 180/pi, 'g-', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Ángulo (grados)');
title('Ángulo del Péndulo');
yline(0, 'k--', 'Vertical');
grid on;

subplot(2,2,3);
plot(simOut.tout, simOut.F, 'm-', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Fuerza (N)');
title('Fuerza de Control');
grid on;

subplot(2,2,4);
axis off;
text(0.1, 0.9, '📊 RESULTADOS', 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.75, sprintf('Coste final: %.4e', f), 'FontSize', 10);
text(0.1, 0.65, sprintf('Kp_{pos}: %.4f', Kp), 'FontSize', 10);
text(0.1, 0.55, sprintf('Kd_{pos}: %.4f', Kd), 'FontSize', 10);
text(0.1, 0.45, sprintf('K_{angle}: %.4f', K_angle), 'FontSize', 10);
text(0.1, 0.35, sprintf('Evaluaciones: %d', nf), 'FontSize', 10);
text(0.1, 0.25, sprintf('Tiempo: %.1f min', tiempo_total/60), 'FontSize', 10);
text(0.1, 0.10, '✅ Optimización completada', 'FontSize', 10, 'Color', 'g');

fprintf('\n✅ ¡Optimización finalizada!\n');
fprintf('=======================================================\n');