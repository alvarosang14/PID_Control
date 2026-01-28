function F = hydraulic_tracklsq(pid, y)
% HYDRAULIC_TRACKLSQ
% Función de coste
% Criterio: ITAE + esfuerzo de control + tiempo de asentamiento

%% configuracion
LOG = true;
PENALTY = 1e6;
model_name = 'SingleActingCylinderWith3WayValve';

if LOG
    fprintf('[COST] Evaluando PID: Kp=%.3f Ki=%.3f Kd=%.3f\n', pid);
end

%% parametros
Kp = pid(1);
Ki = pid(2);
Kd = pid(3);

%% rangos
if (Kp < 0 || Kp > 2000 || Ki < 0 || Ki > 200 || Kd < 0 || Kd > 100)
    if LOG
        fprintf('[COST] PID fuera de rango. Penalización aplicada.\n');
    end
    F = PENALTY;
    return;
end

%% workspace
assignin('base','Kp',Kp);
assignin('base','Ki',Ki);
assignin('base','Kd',Kd);

%% simulacion
try
    if LOG
        fprintf('[COST] Ejecutando simulacion...\n\n');
    end

    simOut = sim(model_name, ...
        'StopTime','5', ...
        'ReturnWorkspaceOutputs','on');

catch
    if LOG
        fprintf('[COST] Error durante la simulación.\n');
    end
    F = PENALTY;
    return;
end

%% posicon
try
    XP = simOut.get('XP_position');
    x  = XP.Data(:);
    t  = XP.Time(:);

    if isempty(x)
        F = PENALTY;
        return;
    end

catch
    F = PENALTY;
    return;
end

%% referencia
reference = evalin('base','XP_ref');
dt = t(2) - t(1);
error = abs(reference - x);

%% coste
F = sum(t .* error) * dt;

if LOG
    fprintf('[COST] ITAE = %.6e\n', F);
end

%% penalizacion coste control
try
    U = simOut.get('XC');
    u = U.Data(:);

    effort = sum(abs(u)) * dt;
    F = F + 1e-4 * effort;

    if LOG
        fprintf('[COST] Esfuerzo de control = %.6e\n', effort);
    end
end

%% Ppenalizacion asentamiento
tol = 0.02 * reference;
idx = find(abs(x - reference) < tol, 1, 'first');

if isempty(idx)
    F = F + 1e-2;
else
    F = F + 1e-3 * t(idx);
end

%% penalizacion fisicas

% no validos
if any(isnan(x)) || any(isinf(x))
    F = PENALTY;
    return;
end

% limite
max_position = max(abs(x));
if max_position > 0.15
    F = F + 1000 * (max_position - 0.15)^2;
end

% oscilaciones
if length(x) > 3
    dx = diff(x);
    sign_changes = sum(abs(diff(sign(dx))) > 1);
    if sign_changes > 10
        F = F + 100 * (sign_changes - 10);
    end
end

%% resultado final
if LOG
    fprintf('[COST] Coste final = %.6e\n', F);
end

end