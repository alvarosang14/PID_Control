%% ========================================================================
%  HYDRAULIC ACTUATOR - SETUP AND DIAGNOSIS
%  Verificación basica
%  ========================================================================

clear; clc; close all;

fprintf('HYDRAULIC ACTUATOR - SETUP AND DIAGNOSIS\n\n');

model_name = 'SingleActingCylinderWith3WayValve';

%% 1. Comprobacion del modelo
try
    load_system(model_name);
    fprintf('Modelo cargado.\n\n');
catch ME
    fprintf('Error al cargar el modelo:\n%s\n', ME.message);
    return;
end

%% 2. Definición de parámetros PID iniciales
Kp = 100;
Ki = 10;
Kd = 5;
XP_ref = 0.03;

assignin('base','Kp',Kp);
assignin('base','Ki',Ki);
assignin('base','Kd',Kd);
assignin('base','XP_ref',XP_ref);

%% 4. Simulación de prueba
fprintf('Ejecutando simulacion de prueba...\n');
set_param(model_name,'StopTime','5');
open_system(model_name);

try
    sim(model_name);
    fprintf('Simulacion completada correctamente.\n');
catch ME
    fprintf('Error durante la simulacion:\n%s\n', ME.message);
end
