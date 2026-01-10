clear all;
clear functions;

% Parámetros encontrados (Iteración 10)
Kp_pos = 1.045882;
Kd_pos = 0.218991;
K_angle = 5000;  % Fijo

% Configurar
assignin('base', 'Kp_pos', Kp_pos);
assignin('base', 'Kd_pos', Kd_pos);

% Verificar K_angle
load_system('rct_pendulum');
set_param('rct_pendulum/Angle Controller', 'Gain', num2str(K_angle));

fprintf('Simulando con:\n');
fprintf('  Kp = %.4f\n', Kp_pos);
fprintf('  Kd = %.4f\n', Kd_pos);
fprintf('  K_angle = %.0f (fijo)\n', K_angle);
fprintf('\n🎬 Observa la animación de Simscape...\n');

% Simular (verás la animación)
simOut = sim('rct_pendulum', 'StopTime', '12');

% Calcular coste
coste = sum(abs(simOut.x - simOut.xref)) + 200*sum(abs(simOut.Theta));

fprintf('\n📊 Resultados:\n');
fprintf('  Coste: %.2f\n', coste);
fprintf('  Posición final: %.4f m\n', simOut.x(end));
fprintf('  Ángulo máximo: %.2f grados\n', max(abs(simOut.Theta))*180/pi);
fprintf('  Ángulo final: %.4f grados\n', simOut.Theta(end)*180/pi);

% Graficar
figure('Position', [100 100 1200 500]);

subplot(1,3,1);
plot(simOut.tout, simOut.xref, 'b--', simOut.tout, simOut.x, 'r-', 'LineWidth', 2);
legend('Referencia', 'Posición');
xlabel('Tiempo (s)'); ylabel('Posición (m)');
title(sprintf('Kp=%.3f, Kd=%.3f | Coste=%.0f', Kp_pos, Kd_pos, coste));
grid on;

subplot(1,3,2);
plot(simOut.tout, simOut.Theta*180/pi, 'g-', 'LineWidth', 2);
xlabel('Tiempo (s)'); ylabel('Ángulo (°)');
title('Ángulo del Péndulo');
yline(0, 'k--', 'Vertical'); grid on;

subplot(1,3,3);
plot(simOut.tout, simOut.F, 'm-', 'LineWidth', 2);
xlabel('Tiempo (s)'); ylabel('Fuerza (N)');
title('Señal de Control');
grid on;