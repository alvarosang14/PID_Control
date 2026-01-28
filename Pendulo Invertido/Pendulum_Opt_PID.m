% =========================================================================
% OPTIMIZACIÓN DEL PÉNDULO INVERTIDO - PID (Kp, Ki, Kd)
% =========================================================================
% K_angle fijo
% Differential Evolution optimiza Kp_pos, Ki_pos, Kd_pos
% =========================================================================

clear all;
close all;
clc;

% -------------------------------------------------------------------------
% CONFIGURACIÓN
% -------------------------------------------------------------------------
K_angle_fijo = 1000;

fprintf('OPTIMIZACIÓN PÉNDULO INVERTIDO - PID\n');
fprintf('K_angle fijo = %.0f\n\n', K_angle_fijo);

% -------------------------------------------------------------------------
% PARÁMETROS DE OPTIMIZACIÓN (DE)
% -------------------------------------------------------------------------
VTR = 1e-6;
D = 3;                  % Kp, Ki, Kd

XVmin = [0,   0,   0];   % [Kp_min, Ki_min, Kd_min]
XVmax = [20,  10,  15];  % [Kp_max, Ki_max, Kd_max]

y = [0 0 0];

NP = 60;
itermax = 40;
F = 0.7;
CR = 0.5;
strategy = 7;
refresh = 5;

% -------------------------------------------------------------------------
% CARGAR MODELO
% -------------------------------------------------------------------------
load_system('rct_pendulum');

set_param('rct_pendulum/Angle Controller', 'Gain', num2str(K_angle_fijo));
save_system('rct_pendulum');

fprintf('Modelo cargado. Iniciando optimización...\n\n');

% -------------------------------------------------------------------------
% OPTIMIZACIÓN
% -------------------------------------------------------------------------
tic;
[x, f, nf] = devec3('pendulum_cost_PID', ...
    VTR, D, XVmin, XVmax, y, NP, itermax, F, CR, strategy, refresh);
tiempo_total = toc;

% -------------------------------------------------------------------------
% RESULTADOS
% -------------------------------------------------------------------------
Kp_opt = x(1);
Ki_opt = x(2);
Kd_opt = x(3);

fprintf('\nRESULTADOS:\n');
fprintf('Kp = %.6f\n', Kp_opt);
fprintf('Ki = %.6f\n', Ki_opt);
fprintf('Kd = %.6f\n', Kd_opt);
fprintf('Coste = %.4e\n', f);
fprintf('Evaluaciones = %d\n', nf);
fprintf('Tiempo = %.1f s\n\n', tiempo_total);

% -------------------------------------------------------------------------
% SIMULACIÓN CON PARÁMETROS ÓPTIMOS
% -------------------------------------------------------------------------
assignin('base','Kp_pos',Kp_opt);
assignin('base','Ki_pos',Ki_opt);
assignin('base','Kd_pos',Kd_opt);

simOut = sim('rct_pendulum','StopTime','12');

% -------------------------------------------------------------------------
% GRÁFICAS
% -------------------------------------------------------------------------
figure('Position',[100 100 1400 800]);

subplot(2,2,1)
plot(simOut.tout, simOut.xref,'k--', simOut.tout, simOut.x,'b','LineWidth',1.8)
grid on
xlabel('Tiempo (s)')
ylabel('Posición (m)')
title('Seguimiento de posición')
legend('Referencia','x')

subplot(2,2,2)
plot(simOut.tout, simOut.Theta*180/pi,'r','LineWidth',1.8)
grid on
xlabel('Tiempo (s)')
ylabel('Ángulo (deg)')
title('Ángulo del péndulo')

subplot(2,2,3)
plot(simOut.tout, simOut.F,'m','LineWidth',1.8)
grid on
xlabel('Tiempo (s)')
ylabel('Fuerza (N)')
title('Señal de control')

subplot(2,2,4)
error_pos = simOut.xref - simOut.x;
plot(simOut.tout, error_pos,'k','LineWidth',1.5)
grid on
xlabel('Tiempo (s)')
ylabel('Error (m)')
title('Error de posición')

sgtitle('Péndulo invertido - PID optimizado con Differential Evolution');

% -------------------------------------------------------------------------
% GUARDAR RESULTADOS
% -------------------------------------------------------------------------
results.Kp = Kp_opt;
results.Ki = Ki_opt;
results.Kd = Kd_opt;
results.K_angle = K_angle_fijo;
results.cost = f;
results.nfeval = nf;
results.time = tiempo_total;
results.sim.time = simOut.tout;
results.sim.x = simOut.x;
results.sim.xref = simOut.xref;
results.sim.theta = simOut.Theta;
results.sim.force = simOut.F;

save('pendulum_PID_optimization_results.mat','results');

fprintf('Resultados guardados.\n');
