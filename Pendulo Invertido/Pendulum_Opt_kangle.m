% =========================================================================
% OPTIMIZACIÓN DEL PÉNDULO INVERTIDO CON PID (POSITION) + K_angle
% =========================================================================

clear;
close all;
clc;

model_name = 'rct_pendulum';

%% ------------------------------------------------------------------------
% PARÁMETROS DE OPTIMIZACIÓN
% -------------------------------------------------------------------------
VTR = 1e-6;          % Criterio de parada
D   = 4;             % [Kp_pos, Ki_pos, Kd_pos, K_angle]

XVmin = [0,   0,   0,     0];
XVmax = [20,  10,  15,  5000];

y = [0 0 0 0];

NP       = 80;
itermax = 50;
F        = 0.8;
CR       = 0.9;
strategy = 7;        % DE/rand/1/bin
refresh  = 5;

%% ------------------------------------------------------------------------
% INICIALIZACIÓN
% -------------------------------------------------------------------------
load_system(model_name);

fprintf('=============================================\n');
fprintf('OPTIMIZACIÓN PÉNDULO INVERTIDO (PID + K_angle)\n');
fprintf('Parámetros: Kp_pos, Ki_pos, Kd_pos, K_angle\n');
fprintf('=============================================\n');

%% ------------------------------------------------------------------------
% EJECUCIÓN DE LA OPTIMIZACIÓN
% -------------------------------------------------------------------------
tic;
[x, f, nf] = devec3( ...
    'pendulum_cost_kangle', ...
    VTR, D, XVmin, XVmax, y, ...
    NP, itermax, F, CR, strategy, refresh);
tiempo_total = toc;

%% ------------------------------------------------------------------------
% RESULTADOS
% -------------------------------------------------------------------------
Kp_pos  = x(1);
Ki_pos  = x(2);
Kd_pos  = x(3);
K_angle = x(4);

fprintf('\nRESULTADOS ÓPTIMOS:\n');
fprintf('Kp_pos  = %.6f\n', Kp_pos);
fprintf('Ki_pos  = %.6f\n', Ki_pos);
fprintf('Kd_pos  = %.6f\n', Kd_pos);
fprintf('K_angle = %.6f\n', K_angle);
fprintf('Coste   = %.6e\n', f);
fprintf('Evaluaciones = %d\n', nf);
fprintf('Tiempo = %.2f s\n', tiempo_total);

%% ------------------------------------------------------------------------
% SIMULACIÓN DE VERIFICACIÓN
% -------------------------------------------------------------------------
assignin('base','Kp_pos',  Kp_pos);
assignin('base','Ki_pos',  Ki_pos);
assignin('base','Kd_pos',  Kd_pos);
assignin('base','K_angle', K_angle);

set_param(model_name,'StopTime','12');
simOut = sim(model_name,'ReturnWorkspaceOutputs','on');

%% ------------------------------------------------------------------------
% GRÁFICAS
% -------------------------------------------------------------------------
figure;

subplot(2,2,1);
plot(simOut.tout, simOut.xref, 'k--', simOut.tout, simOut.x, 'b','LineWidth',1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('Posición (m)');
title('Posición del carro');

subplot(2,2,2);
plot(simOut.tout, simOut.Theta*180/pi,'r','LineWidth',1.5);
yline(0,'k--');
grid on;
xlabel('Tiempo (s)');
ylabel('Ángulo (deg)');
title('Ángulo del péndulo');

subplot(2,2,3);
plot(simOut.tout, simOut.F,'m','LineWidth',1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('Fuerza (N)');
title('Señal de control');

subplot(2,2,4);
axis off;
text(0.1,0.8,sprintf('Coste: %.3e',f));
text(0.1,0.6,sprintf('Kp = %.3f',Kp_pos));
text(0.1,0.5,sprintf('Ki = %.3f',Ki_pos));
text(0.1,0.4,sprintf('Kd = %.3f',Kd_pos));
text(0.1,0.3,sprintf('K_{angle} = %.1f',K_angle));
