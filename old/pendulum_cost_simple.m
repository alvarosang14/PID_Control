function F = pendulum_cost_simple(params, y)
% Función de coste SIMPLIFICADA para el péndulo invertido
% Esta versión es más adaptable - ajústala según tu modelo
%
% INSTRUCCIONES DE ADAPTACIÓN:
% 1. Verifica los nombres exactos de tus bloques en Simulink
% 2. Ajusta cómo se configuran los parámetros del controlador
% 3. Verifica qué señales están en yout y en qué columnas

% Extraer parámetros (ajusta según cuántos necesites)
Kp_pos = params(1);
Kd_pos = params(2);

% Si tienes más parámetros, descoméntalos:
% K1_angle = params(3);
% K2_angle = params(4);

% Validación básica de rangos
if any(params < 0)
    F = 1e10;
    return;
end

try
    % ============================================
    % CONFIGURA AQUÍ TU MODELO
    % ============================================
    % Opción 1: Si tienes un bloque PID estándar
    % set_param('rct_pendulum/PID_Position', 'P', num2str(Kp_pos));
    % set_param('rct_pendulum/PID_Position', 'D', num2str(Kd_pos));
    
    % Opción 2: Si tienes ganancias separadas
    % set_param('rct_pendulum/Kp_gain', 'Gain', num2str(Kp_pos));
    % set_param('rct_pendulum/Kd_gain', 'Gain', num2str(Kd_pos));
    
    % Opción 3: Si usas variables del workspace
    assignin('base', 'Kp_pos', Kp_pos);
    assignin('base', 'Kd_pos', Kd_pos);
    % Y en tu modelo configuras los bloques para usar estas variables
    
    % Simular
    simOut = sim('rct_pendulum', 'StopTime', '20');
    
    % ============================================
    % EXTRAE LAS SEÑALES QUE NECESITAS
    % ============================================
    % Opción A: Si usas To Workspace blocks
    pos_ref = evalin('base', 'pos_ref');     % Referencia de posición
    pos_actual = evalin('base', 'pos_actual'); % Posición real
    angulo = evalin('base', 'angulo');       % Ángulo del péndulo
    
    % Opción B: Si usas Simscape o tienes todo en yout
    % En este caso, identifica qué columna es cada señal
    % yout = simOut.yout;  % o similar
    
    % ============================================
    % CALCULA EL COSTE
    % ============================================
    % Error de posición
    pos_error = sum(abs(pos_ref - pos_actual));
    
    % Desviación angular (lo más importante para el péndulo)
    angle_error = sum(abs(angulo));
    
    % Sobrepaso y oscilaciones
    overshoot = max(pos_actual) - max(pos_ref);
    if overshoot < 0, overshoot = 0; end
    
    % Función de coste combinada
    F = pos_error + 20*angle_error + 5*overshoot;
    
    % Mostrar progreso ocasionalmente
    persistent call_count;
    if isempty(call_count), call_count = 0; end
    call_count = call_count + 1;
    if mod(call_count, 50) == 0
        fprintf('  [Eval %d] Coste: %.4f | Kp=%.2f, Kd=%.2f\n', ...
                call_count, F, Kp_pos, Kd_pos);
    end
    
catch ME
    fprintf('ERROR en simulación: %s\n', ME.message);
    F = 1e10;
end

end
