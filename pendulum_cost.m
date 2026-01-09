function F = pendulum_cost(params, y)
% Función de coste para péndulo invertido con 3 parámetros
% params = [Kp_pos, Kd_pos, K_angle]

persistent eval_count best_cost;
if isempty(eval_count)
    eval_count = 0;
    best_cost = inf;
end
eval_count = eval_count + 1;

Kp = params(1);
Kd = params(2);
K_angle = params(3);

% Validación (debe coincidir con XVmin y XVmax)
if (Kp < 0 || Kp > 15 || Kd < 0 || Kd > 8 || K_angle < 0 || K_angle > 100)
    F = 10e6;
else
    % Configurar parámetros
    assignin('base', 'Kp_pos', Kp);
    assignin('base', 'Kd_pos', Kd);
    assignin('base', 'K_angle', K_angle);
    
    % Simular
    simOut = sim('rct_pendulum', 'StopTime', '12');
    
    % Extraer señales (son arrays directos)
    x = simOut.x;
    xref = simOut.xref;
    theta = simOut.Theta;
    
    % Función de coste
    F = sum(abs(x - xref)) + 50*sum(abs(theta));
    
    % Mostrar progreso
    if F < best_cost
        best_cost = F;
        fprintf('  🎯 Eval %4d - MEJOR: %.4f | Kp=%.3f, Kd=%.3f, Ka=%.3f\n', ...
                eval_count, F, Kp, Kd, K_angle);
    elseif mod(eval_count, 10) == 0
        fprintf('  ... Eval %4d - Coste: %.4f | Kp=%.3f, Kd=%.3f, Ka=%.3f\n', ...
                eval_count, F, Kp, Kd, K_angle);
    end
    
end