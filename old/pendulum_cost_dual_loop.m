function F = pendulum_cost_dual_loop(params, y)
% Función de coste para péndulo con DOBLE LAZO
% Lazo interno: Controlador de espacio de estados (ángulo)
% Lazo externo: Controlador PD (posición)
%
% params puede ser:
% - Versión 1: [Kp_pos, Kd_pos] (2 parámetros, si el controlador de ángulo es fijo)
% - Versión 2: [Kp_pos, Kd_pos, K_state1, K_state2, K_state3, K_state4] (6 params)

% Extraer parámetros
num_params = length(params);

if num_params == 2
    % Solo optimizamos el PD de posición
    Kp_pos = params(1);
    Kd_pos = params(2);
    optimize_state_controller = false;
    
elseif num_params >= 4
    % Optimizamos PD + controlador de estado
    Kp_pos = params(1);
    Kd_pos = params(2);
    K_state = params(3:end);  % Ganancias del controlador de estado
    optimize_state_controller = true;
    
else
    error('Número de parámetros no válido: debe ser 2 o 4+');
end

% Límites de validación (ajusta según tu sistema)
if Kp_pos < 0 || Kp_pos > 20 || Kd_pos < 0 || Kd_pos > 10
    F = 1e10;
    return;
end

if optimize_state_controller && any(K_state < 0)
    F = 1e10;
    return;
end

try
    %% Configurar controlador PD de posición
    % OPCIÓN 1: Si usas un bloque PID estándar de Simulink
    % set_param('rct_pendulum/Position Controller', 'P', num2str(Kp_pos));
    % set_param('rct_pendulum/Position Controller', 'I', '0');  % Sin acción integral
    % set_param('rct_pendulum/Position Controller', 'D', num2str(Kd_pos));
    
    % OPCIÓN 2: Si usas variables del workspace
    assignin('base', 'Kp_pos', Kp_pos);
    assignin('base', 'Kd_pos', Kd_pos);
    
    %% Configurar controlador de espacio de estados (si se optimiza)
    if optimize_state_controller
        % Para un controlador de estado x_dot = Ax + Bu, u = -Kx
        % Típicamente K es un vector de ganancias [k1, k2, k3, k4]
        % correspondientes a [x, x_dot, theta, theta_dot]
        
        K_matrix = K_state;  % Vector de ganancias
        assignin('base', 'K_state', K_matrix);
        
        % O si tienes ganancias individuales:
        % assignin('base', 'K1', K_state(1));
        % assignin('base', 'K2', K_state(2));
        % assignin('base', 'K3', K_state(3));
        % assignin('base', 'K4', K_state(4));
    end
    
    %% Simular el sistema
    sim_time = 12;  % 12 segundos (configurado en el modelo)
    simOut = sim('rct_pendulum', 'StopTime', num2str(sim_time));
    
    %% Extraer señales
    % Método 1: Desde workspace (si usas To Workspace)
    try
        t = evalin('base', 'tout');
        x_ref = evalin('base', 'x_ref');      % Referencia de posición
        x = evalin('base', 'x');              % Posición del carro
        theta = evalin('base', 'theta');      % Ángulo del péndulo (rad)
        F_control = evalin('base', 'F');      % Fuerza aplicada
    catch
        % Método 2: Si las señales están en simOut
        % Adapta según tu configuración
        warning('No se pudieron leer las señales del workspace');
        F = 1e10;
        return;
    end
    
    %% Calcular componentes del coste
    
    % 1. Error de seguimiento de posición (IAE - Integral Absolute Error)
    position_error = trapz(t, abs(x - x_ref));
    
    % 2. Desviación angular (MUY IMPORTANTE - debe mantenerse cerca de 0)
    % Usar rad² para penalizar más las grandes desviaciones
    angle_error = trapz(t, theta.^2);
    
    % 3. Esfuerzo de control (limitar fuerza excesiva)
    control_effort = trapz(t, F_control.^2);
    
    % 4. Velocidad angular (penalizar oscilaciones)
    theta_dot = diff(theta)./diff(t);
    oscillation_penalty = sum(abs(theta_dot));
    
    % 5. Tiempo de establecimiento (settling time)
    % Penalizar si hay mucha variación en la última parte
    last_20_percent = round(0.8*length(t)):length(t);
    settling_error = std(x(last_20_percent));
    
    % 6. Sobrepaso (overshoot)
    if max(x_ref) > 0
        overshoot = (max(x) - max(x_ref)) / max(x_ref);
        overshoot = max(0, overshoot);  % Solo penalizar sobrepasos positivos
    else
        overshoot = 0;
    end
    
    % 7. Penalización por inestabilidad (si el péndulo se cae)
    max_angle_deg = max(abs(theta)) * 180/pi;
    if max_angle_deg > 30  % Si se desvía más de 30 grados
        instability_penalty = 1000 * max_angle_deg;
    else
        instability_penalty = 0;
    end
    
    %% Función de coste combinada con pesos
    % AJUSTA ESTOS PESOS según la importancia de cada objetivo
    w1 = 1.0;      % Seguimiento de posición
    w2 = 50.0;     % Estabilización angular (¡MUY IMPORTANTE!)
    w3 = 0.01;     % Esfuerzo de control
    w4 = 5.0;      % Oscilaciones
    w5 = 10.0;     % Settling time
    w6 = 15.0;     % Sobrepaso
    w7 = 1.0;      % Inestabilidad
    
    F = w1 * position_error + ...
        w2 * angle_error + ...
        w3 * control_effort + ...
        w4 * oscillation_penalty + ...
        w5 * settling_error + ...
        w6 * overshoot + ...
        w7 * instability_penalty;
    
    % Mostrar progreso cada N evaluaciones
    persistent eval_counter last_best_cost;
    if isempty(eval_counter)
        eval_counter = 0;
        last_best_cost = inf;
    end
    eval_counter = eval_counter + 1;
    
    if F < last_best_cost
        last_best_cost = F;
        fprintf('  🎯 [Eval %4d] Nuevo mejor coste: %.4e | Kp=%.3f, Kd=%.3f', ...
                eval_counter, F, Kp_pos, Kd_pos);
        if optimize_state_controller
            fprintf(' | K=[');
            fprintf('%.2f ', K_state);
            fprintf(']\n');
        else
            fprintf('\n');
        end
    elseif mod(eval_counter, 100) == 0
        fprintf('  ⏳ [Eval %4d] Coste: %.4e\n', eval_counter, F);
    end
    
catch ME
    fprintf('❌ ERROR en simulación: %s\n', ME.message);
    fprintf('   Parámetros: Kp=%.3f, Kd=%.3f\n', Kp_pos, Kd_pos);
    F = 1e10;
end

end