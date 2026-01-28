function F = pendulum_cost_PID(params, y)
% Coste para péndulo invertido con PID de posición + K_angle
% params = [Kp_pos, Ki_pos, Kd_pos, K_angle]

persistent eval_count best_cost
if isempty(eval_count)
    eval_count = 0;
    best_cost = inf;
end
eval_count = eval_count + 1;

%% parametros
Kp_pos  = params(1);
Ki_pos  = params(2);
Kd_pos  = params(3);
K_angle = params(4);

%% rangos
if (Kp_pos  < 0 || Kp_pos  > 20 || ...
    Ki_pos  < 0 || Ki_pos  > 10 || ...
    Kd_pos  < 0 || Kd_pos  > 15 || ...
    K_angle < 0 || K_angle > 5000)

    F = 1e6;
    return;
end

%% asignar
assignin('base','Kp_pos',  Kp_pos);
assignin('base','Ki_pos',  Ki_pos);
assignin('base','Kd_pos',  Kd_pos);
assignin('base','K_angle', K_angle);

%% simular
try
    simOut = sim('rct_pendulum', 'StopTime','12', ...
                 'ReturnWorkspaceOutputs','on');
catch
    F = 1e6;
    return;
end

%% extraer
x     = simOut.x(:);        % Posicion del carro
xref  = simOut.xref(:);     % Referencia
theta = simOut.Theta(:);    % Angulo
t     = simOut.tout(:);

if length(t) < 2
    F = 1e6;
    return;
end

dt = t(2) - t(1);

%% Coste
% Error de posicion
e_pos = abs(x - xref);
J_pos = sum(t .* e_pos) * dt;

% Penalización por angulo
J_ang = sum(abs(theta)) * dt;

% Coste total
F = J_pos + 200 * J_ang;

%% Log final
if F < best_cost
    best_cost = F;
    fprintf('BEST | Eval %4d | F %.4f | Kp %.3f Ki %.3f Kd %.3f Ka %.1f\n', ...
        eval_count, F, Kp_pos, Ki_pos, Kd_pos, K_angle);
elseif mod(eval_count, 20) == 0
    fprintf('Eval %4d | F %.4f | Kp %.3f Ki %.3f Kd %.3f Ka %.1f\n', ...
        eval_count, F, Kp_pos, Ki_pos, Kd_pos, K_angle);
end

end
