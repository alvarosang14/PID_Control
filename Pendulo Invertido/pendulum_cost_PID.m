function F = pendulum_cost_PID(params, y)
% Coste para péndulo invertido con PID de posición

persistent eval_count
if isempty(eval_count)
    eval_count = 0;
end
eval_count = eval_count + 1;

Kp = params(1);
Ki = params(2);
Kd = params(3);

PENALTY = 1e6;

if (Kp < 0 || Kp > 20 || Ki < 0 || Ki > 10 || Kd < 0 || Kd > 15)
    F = PENALTY;
    return;
end

assignin('base','Kp_pos',Kp);
assignin('base','Ki_pos',Ki);
assignin('base','Kd_pos',Kd);

try
    simOut = sim('rct_pendulum','StopTime','12');
catch
    F = PENALTY;
    return;
end

x     = simOut.x;
xref  = simOut.xref;
theta = simOut.Theta;
t     = simOut.tout;

dt = t(2) - t(1);

F = sum(abs(x - xref))*dt + 200*sum(abs(theta))*dt;

% cada 10 imprimo
if mod(eval_count,10)==0
    fprintf('Eval %4d | Coste %.4f | Kp %.3f Ki %.3f Kd %.3f\n', ...
            eval_count, F, Kp, Ki, Kd);
end

end
