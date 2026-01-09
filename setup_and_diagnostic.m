% SCRIPT DE DIAGNÓSTICO Y CONFIGURACIÓN INICIAL
% Ejecuta este script ANTES de la optimización para verificar que todo está bien

clear all;
close all;
clc;

fprintf('=======================================================\n');
fprintf('DIAGNÓSTICO DEL MODELO PARA OPTIMIZACIÓN\n');
fprintf('=======================================================\n\n');

%% 1. Verificar que el modelo existe
modelo = 'rct_pendulum';
fprintf('1. Verificando modelo "%s"...\n', modelo);

if exist([modelo '.slx'], 'file') == 4
    fprintf('   ✅ Modelo encontrado: %s.slx\n', modelo);
else
    fprintf('   ❌ ERROR: Modelo no encontrado.\n');
    fprintf('   Asegúrate de que %s.slx está en el path de MATLAB.\n', modelo);
    fprintf('   Path actual: %s\n', pwd);
    return;
end

%% 2. Cargar el modelo
fprintf('\n2. Cargando modelo...\n');
try
    load_system(modelo);
    fprintf('   ✅ Modelo cargado correctamente\n');
catch ME
    fprintf('   ❌ ERROR al cargar: %s\n', ME.message);
    return;
end

%% 3. Listar bloques del modelo
fprintf('\n3. Explorando estructura del modelo...\n');
try
    bloques = find_system(modelo, 'Type', 'Block');
    fprintf('   Total de bloques: %d\n', length(bloques));
    
    % Buscar bloques de control relevantes
    fprintf('\n   Bloques que contienen "Control" o "PID":\n');
    for i = 1:length(bloques)
        if contains(lower(bloques{i}), {'control', 'pid', 'gain', 'controller'})
            fprintf('     - %s\n', bloques{i});
        end
    end
catch ME
    fprintf('   ⚠️  No se pudo listar bloques: %s\n', ME.message);
end

%% 4. Verificar parámetros de simulación
fprintf('\n4. Configuración de simulación:\n');
try
    solver = get_param(modelo, 'Solver');
    stop_time = get_param(modelo, 'StopTime');
    fprintf('   Solver: %s\n', solver);
    fprintf('   Tiempo de simulación: %s segundos\n', stop_time);
    fprintf('   ✅ Configuración OK\n');
catch ME
    fprintf('   ⚠️  Error al leer configuración: %s\n', ME.message);
end

%% 5. Probar simulación básica
fprintf('\n5. Probando simulación básica...\n');

% Valores de prueba
Kp_pos = 5;
Kd_pos = 2;
fprintf('   Usando valores de prueba: Kp=%.1f, Kd=%.1f\n', Kp_pos, Kd_pos);

% Configurar en el workspace
assignin('base', 'Kp_pos', Kp_pos);
assignin('base', 'Kd_pos', Kd_pos);

try
    % Simular
    fprintf('   Ejecutando simulación de 10 segundos...\n');
    tic;
    simOut = sim(modelo, 'StopTime', '10');
    tiempo_sim = toc;
    fprintf('   ✅ Simulación exitosa en %.2f segundos\n', tiempo_sim);
    
    % Verificar si hay variables en el workspace
    fprintf('\n6. Variables exportadas al workspace:\n');
    vars = evalin('base', 'who');
    if ~isempty(vars)
        fprintf('   Variables disponibles:\n');
        for i = 1:length(vars)
            var_size = evalin('base', sprintf('size(%s)', vars{i}));
            fprintf('     - %s [%dx%d]\n', vars{i}, var_size(1), var_size(2));
        end
        fprintf('   ✅ Variables exportadas correctamente\n');
    else
        fprintf('   ⚠️  No hay variables en el workspace.\n');
        fprintf('   Necesitas añadir bloques "To Workspace" en tu modelo.\n');
    end
    
catch ME
    fprintf('   ❌ ERROR en simulación: %s\n', ME.message);
    fprintf('\n   POSIBLES CAUSAS:\n');
    fprintf('   1. Los bloques del modelo no están configurados para usar Kp_pos/Kd_pos\n');
    fprintf('   2. Faltan parámetros necesarios en el workspace\n');
    fprintf('   3. Hay un error en el modelo\n');
end

%% 7. Recomendaciones
fprintf('\n=======================================================\n');
fprintf('RECOMENDACIONES PARA LA OPTIMIZACIÓN\n');
fprintf('=======================================================\n');

fprintf('\n📌 ANTES de ejecutar la optimización, asegúrate de:\n\n');

fprintf('1. CONFIGURAR BLOQUES TO WORKSPACE:\n');
fprintf('   - x_ref (referencia de posición)\n');
fprintf('   - x (posición actual del carro)\n');
fprintf('   - theta (ángulo del péndulo)\n');
fprintf('   - F (fuerza de control)\n');
fprintf('   - tout (vector de tiempo)\n\n');

fprintf('2. CONFIGURAR PARÁMETROS AJUSTABLES:\n');
fprintf('   Opción A: Usa variables del workspace (Kp_pos, Kd_pos)\n');
fprintf('   Opción B: Configura con set_param en la función de coste\n\n');

fprintf('3. PROBAR MANUALMENTE:\n');
fprintf('   - Simula con diferentes valores de Kp y Kd\n');
fprintf('   - Verifica que el péndulo no se cae\n');
fprintf('   - Ajusta los límites de búsqueda (XVmin, XVmax)\n\n');

fprintf('4. AJUSTAR LA FUNCIÓN DE COSTE:\n');
fprintf('   - Identifica qué columnas de yout contienen cada señal\n');
fprintf('   - Ajusta los pesos según la importancia de cada objetivo\n');
fprintf('   - Prioriza MUCHO la estabilización del péndulo (peso ~50)\n\n');

fprintf('=======================================================\n');
fprintf('SIGUIENTE PASO: Edita pendulum_cost_simple.m\n');
fprintf('=======================================================\n\n');

%% 8. Sugerencia de valores iniciales
fprintf('💡 VALORES SUGERIDOS basados en el ejemplo de systune:\n\n');
fprintf('   Kp_posicion: ~6.11 → Rango recomendado [0, 15]\n');
fprintf('   Kd_posicion: ~2.19 → Rango recomendado [0, 8]\n\n');

fprintf('   Parámetros DE recomendados para empezar:\n');
fprintf('   - NP (población): 40-60\n');
fprintf('   - itermax: 20-30 (para pruebas), 50-100 (optimización final)\n');
fprintf('   - F (diferencial): 0.8\n');
fprintf('   - CR (cruce): 0.9\n');
fprintf('   - strategy: 7 (DE/rand/1/bin)\n\n');

fprintf('=======================================================\n');
fprintf('¡Listo! Ahora puedes editar las funciones de coste.\n');
fprintf('=======================================================\n');
