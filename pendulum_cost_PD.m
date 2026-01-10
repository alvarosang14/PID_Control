function F = pendulum_cost_PD(params, y)
% Función de coste para péndulo invertido - SOLO PD
% K_angle está FIJO en el modelo (como SOF en el helicóptero)
% params = [Kp_pos, Kd_pos]

persistent eval_count best_cost;
if isempty(eval_count)
    eval_count = 0;
    best_cost = inf;
end
eval_count = eval_count + 1;

Kp = params(1);
Kd = params(2);

% Validación (debe coincidir con XVmin y XVmax)
if (Kp < 0 || Kp > 20 || Kd < 0 || Kd > 15)
    F = 10e6;
else
    % Configurar solo PD (K_angle ya está fijo en el modelo)
    assignin('base', 'Kp_pos', Kp);
    assignin('base', 'Kd_pos', Kd);
    
    % Simular
    simOut = sim('rct_pendulum', 'StopTime', '12');
    
    % Extraer señales (son arrays directos)
    x = simOut.x;
    xref = simOut.xref;
    theta = simOut.Theta;
    
    % Función de coste
    F = sum(abs(x - xref)) + 200*sum(abs(theta));
    
    % Mostrar progreso
    if F < best_cost
        best_cost = F;
        fprintf('  🎯 [BEST] Eval %4d - Cost: %.4f | Kp=%.3f, Kd=%.3f\n', ...
                eval_count, F, Kp, Kd);
    elseif mod(eval_count, 10) == 0
        fprintf('  ... Eval %4d - Cost: %.4f | Kp=%.3f, Kd=%.3f\n', ...
                eval_count, F, Kp, Kd);
    end
    
end
