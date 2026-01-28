%% ========================================================================
%  OPTIMIZACIÓN PID - SISTEMA HIDRÁULICO CON DIFFERENTIAL EVOLUTION
%  Script principal
%  ========================================================================
clear; clc; close all;

model_name = 'SingleActingCylinderWith3WayValve';

%% Cargar modelo
load_system(model_name);

%% Parámetros DE
VTR = 1.e-6;
D = 3;

XVmin = [10,   1,    0.1];
XVmax = [1000, 100,  50];

y = [0, 0, 0];
NP = 50;
itermax = 15;
F = 0.8;
CR = 0.8;
strategy = 7;
refresh = 5;

%% Referencia
XP_ref = 0.03;
assignin('base','XP_ref',XP_ref);

%% Optimizacion
tic;
[x, f, nf] = devec3('hydraulic_tracklsq', ...
    VTR, D, XVmin, XVmax, y, NP, itermax, F, CR, strategy, refresh);
tiempo_opt = toc;

%% Resultados
Kp_opt = x(1);
Ki_opt = x(2);
Kd_opt = x(3);

fprintf('Kp = %.4f\n', Kp_opt);
fprintf('Ki = %.4f\n', Ki_opt);
fprintf('Kd = %.4f\n', Kd_opt);
fprintf('Coste = %.6e\n', f);

%% Simular con optimo
Kp = Kp_opt;
Ki = Ki_opt;
Kd = Kd_opt;

set_param(model_name, 'StopTime', '5');
simOut_opt = sim(model_name,'ReturnWorkspaceOutputs','on');

%% Guardar resultados y señales
XP = simOut_opt.get('XP_position');

resultados.Kp_opt = Kp_opt;
resultados.Ki_opt = Ki_opt;
resultados.Kd_opt = Kd_opt;
resultados.coste = f;
resultados.nfeval = nf;
resultados.tiempo = tiempo_opt;
resultados.XP_time = XP.Time(:);
resultados.XP_data = XP.Data(:);
resultados.XP_ref = XP_ref;

save('hydraulic_optimization_results.mat','resultados');

fprintf('Resultados guardados en hydraulic_optimization_results.mat\n');

plot_hydraulic_results