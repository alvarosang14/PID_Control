%% ========================================================================
%  GRÁFICAS DE RESULTADOS PID HIDRÁULICO
%  ========================================================================
clear; clc; close all;

load('hydraulic_optimization_results.mat');

t = resultados.XP_time;
x = resultados.XP_data;
ref = resultados.XP_ref;

%% Grafica posición
figure;
plot(t, x*1000, 'b', 'LineWidth', 2); hold on;
yline(ref*1000, 'k--', 'Referencia');
grid on;
xlabel('Tiempo (s)');
ylabel('Posición (mm)');
title('Respuesta del cilindro - PID óptimo');
legend('XP', 'Referencia');

%% Error
figure;
e = (ref - x)*1000;
plot(t, e, 'r', 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)');
ylabel('Error (mm)');
title('Error de seguimiento');

%% Métricas básicas
final_value = mean(x(end-10:end));
steady_state_error = abs(ref - final_value)*1000;
overshoot = max(0, (max(x)-ref)/ref*100);

fprintf('Error estacionario (mm): %.4f\n', steady_state_error);
fprintf('Overshoot (%%): %.2f\n', overshoot);
