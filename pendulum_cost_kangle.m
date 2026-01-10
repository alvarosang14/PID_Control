function F = pendulum_cost(params, y)
% Cost function for inverted pendulum with 3 parameters
% params = [Kp_pos, Kd_pos, K_angle]
%
% Returns: F - cost value (lower is better)
% set_param('rct_pendulum', 'SimMechanicsOpenEditorOnUpdate', 'on');
%save_system('rct_pendulum');
persistent eval_count best_cost;
if isempty(eval_count)
    eval_count = 0;
    best_cost = inf;
end
eval_count = eval_count + 1;

% Extract parameters
Kp = params(1);
Kd = params(2);
K_angle = params(3);

% Parameter validation (must match XVmin and XVmax bounds)
if (Kp < 0 || Kp > 20 || Kd < 0 || Kd > 15 || K_angle < 0 || K_angle > 5000)
    F = 10e6;
    return;
end

% Set parameters in base workspace
assignin('base', 'Kp_pos', Kp);
assignin('base', 'Kd_pos', Kd);
assignin('base', 'K_angle', K_angle);

% Run simulation
simOut = sim('rct_pendulum', 'StopTime', '12');

% Extract signals
x = simOut.x;           % Cart position
xref = simOut.xref;     % Reference position
theta = simOut.Theta;   % Pendulum angle

% Calculate cost function
% Minimize position error and angle deviation
%F = sum(abs(x - xref)) + 50*sum(abs(theta));
F = sum(abs(x - xref)) + 200*sum(abs(theta));

% Display progress
if F < best_cost
    best_cost = F;
    fprintf('  [BEST] Eval %4d - Cost: %.4f | Kp=%.3f, Kd=%.3f, Ka=%.3f\n', ...
            eval_count, F, Kp, Kd, K_angle);
elseif mod(eval_count, 10) == 0
    fprintf('         Eval %4d - Cost: %.4f | Kp=%.3f, Kd=%.3f, Ka=%.3f\n', ...
            eval_count, F, Kp, Kd, K_angle);
end