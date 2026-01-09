clear all;
load_system('rct_pendulum.slx');

Kp_pos = 6;
Kd_pos = 2;
assignin('base', 'Kp_pos', Kp_pos);
assignin('base', 'Kd_pos', Kd_pos);

% Simular
simOut = sim('rct_pendulum', 'StopTime', '12');

% Ver QUÉ variables se exportaron
vars = evalin('base', 'who');
fprintf('Variables exportadas:\n');
for i = 1:length(vars)
    fprintf('  %s\n', vars{i});
end

% Verificar si las señales están ahí
if evalin('base', 'exist(''x'', ''var'')')
    fprintf('\n✅ Variable "x" encontrada!\n');
    x = evalin('base', 'x');
    fprintf('   Tamaño: [%d x %d]\n', size(x,1), size(x,2));
end

if evalin('base', 'exist(''Theta'', ''var'')')
    fprintf('✅ Variable "Theta" encontrada!\n');
    theta = evalin('base', 'Theta');
    fprintf('   Tamaño: [%d x %d]\n', size(theta,1), size(theta,2));
end

if evalin('base', 'exist(''xref'', ''var'')')
    fprintf('✅ Variable "xref" encontrada!\n');
end

if evalin('base', 'exist(''F'', ''var'')')
    fprintf('✅ Variable "F" encontrada!\n');
end