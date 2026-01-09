function F = pendulum_cost(params, y)
% Función de coste para optimización del péndulo invertido
% params = [Kp_pos, Kd_pos, K1_angle, K2_angle]
% donde:
%   Kp_pos, Kd_pos: ganancias del controlador PD de posición
%   K1_angle, K2_angle: ganancias del controlador de ángulo

% Extraer parámetros
Kp_pos = params(1);
Kd_pos = params(2);
K1_angle = params(3);
K2_angle = params(4);

% Validación de rangos (ajusta según necesites)
if (Kp_pos < 0 || Kp_pos > 20 || ...
    Kd_pos < 0 || Kd_pos > 10 || ...
    K1_angle < 0 || K1_angle > 100 || ...
    K2_angle < 0 || K2_angle > 100)
    F = 1e10;  % Penalización muy alta si está fuera de rango
    return;
end

try
    % Configurar los parámetros en el modelo de Simulink
    % NOTA: Ajusta estos nombres según tu modelo real
    set_param('rct_pendulum/Position Controller', 'P', num2str(Kp_pos));
    set_param('rct_pendulum/Position Controller', 'D', num2str(Kd_pos));
    
    % Para el controlador de ángulo (espacio de estados)
    % Esto depende de cómo esté implementado en tu modelo
    % Puede ser que necesites configurar las matrices A, B, C, D
    set_param('rct_pendulum/Angle Controller', 'Gain1', num2str(K1_angle));
    set_param('rct_pendulum/Angle Controller', 'Gain2', num2str(K2_angle));
    
    % Simular el sistema
    [tout, xout, yout] = sim('rct_pendulum', [0 20]);
    
    drawnow;
    
    % Definir la función de coste multiobjetivo
    % yout debería contener: posición_carro, ángulo_péndulo, fuerza, etc.
    
    % Coste 1: Error de seguimiento de posición
    if size(yout, 2) >= 2
        position_error = sum(abs(yout(:,1) - yout(:,2)));  % Referencia vs real
    else
        position_error = 0;
    end
    
    % Coste 2: Desviación angular (mantener péndulo vertical)
    % Asumiendo que la columna 3 o similar tiene el ángulo
    if size(yout, 2) >= 3
        angle_deviation = sum(abs(yout(:,3)));  % Queremos ángulo = 0
    else
        angle_deviation = 0;
    end
    
    % Coste 3: Esfuerzo de control
    if size(yout, 2) >= 4
        control_effort = 0.01 * sum(abs(yout(:,4)));
    else
        control_effort = 0;
    end
    
    % Coste 4: Tiempo de establecimiento (penalizar oscilaciones)
    settling_penalty = 0;
    if size(yout, 2) >= 2
        % Penalizar si no se estabiliza en el último 25% del tiempo
        last_quarter = round(0.75 * length(tout)):length(tout);
        settling_error = std(yout(last_quarter, 2));
        settling_penalty = 10 * settling_error;
    end
    
    % Función de coste combinada (ponderada según importancia)
    F = 1.0 * position_error + ...      % Seguimiento de posición
        16.0 * angle_deviation + ...     % Mantener péndulo vertical (MUY importante)
        0.1 * control_effort + ...       % Limitar esfuerzo de control
        5.0 * settling_penalty;          % Penalizar oscilaciones
    
catch ME
    % Si hay algún error en la simulación, devolver coste muy alto
    fprintf('Error en simulación: %s\n', ME.message);
    F = 1e10;
end

end
