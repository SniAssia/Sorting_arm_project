function seven_build_simulink()
%SEVEN_BUILD_SIMULINK Generate a .slx for the assumed first-order servo model.
% This uses standard Simulink, not Control System Toolbox.
% DO NOT claim tau has been identified from experimental measurements.
rootDir=fileparts(mfilename('fullpath'));
outDir=fullfile(rootDir,'outputs');
sourceFile=fullfile(outDir,'seven_simulink_inputs.mat');
if ~exist(sourceFile,'file')
    error('Run run_SEVEN_project first to generate commanded joint trajectories.');
end
% Do not infer installation/licensing from exist('new_system','file').
% Simulink APIs may be resolved differently across MATLAB installations.
% Instead, let the actual Simulink API report any environment problem.
S=load(sourceFile);
assert(S.seven_simulink_tau>0,'Servo lag time constant must be positive');

% From Workspace accepts [time, scalar value] pairs. We use zero-order hold.
assignin('base','seven_base_cmd',[S.seven_simulink_time,S.seven_simulink_Q(:,1)]);
assignin('base','seven_shoulder_cmd',[S.seven_simulink_time,S.seven_simulink_Q(:,2)]);
assignin('base','seven_elbow_cmd',[S.seven_simulink_time,S.seven_simulink_Q(:,3)]);
assignin('base','seven_tau',S.seven_simulink_tau);

model='SEVEN_Servo_Tracking';
try
    if bdIsLoaded(model), close_system(model,0); end
    new_system(model);
    open_system(model);
catch ME
    fprintf(2, ['Unable to initialize Simulink.\n' ...
        'Run: ver; which new_system -all; license(''test'',''Simulink'')\n' ...
        'Original Simulink error: %s\n'], ME.message);
    rethrow(ME);
end
set_param(model,'Solver','ode45','StopTime',num2str(S.seven_simulink_time(end)));
axisNames={'Base','Shoulder','Elbow'};
workspaceVars={'seven_base_cmd','seven_shoulder_cmd','seven_elbow_cmd'};
for k=1:3
    y=70+(k-1)*195;
    in=[model '/' axisNames{k} '_Command'];
    lag=[model '/' axisNames{k} '_Lag'];
    mux=[model '/' axisNames{k} '_Compare'];
    scope=[model '/' axisNames{k} '_Scope'];
    add_block('simulink/Sources/From Workspace',in, ...
        'Position',[45 y 205 y+45]);
    set_param(in,'VariableName',workspaceVars{k},'Interpolate','off', ...
        'OutputAfterFinalValue','Holding final value');
    add_block('simulink/Continuous/Transfer Fcn',lag, ...
        'Numerator','1','Denominator','[seven_tau 1]', ...
        'Position',[270 y+75 410 y+115]);
    add_block('simulink/Signal Routing/Mux',mux, ...
        'Inputs','2','Position',[465 y+10 485 y+85]);
    add_block('simulink/Sinks/Scope',scope, ...
        'Position',[560 y+25 650 y+70]);
    add_line(model,[axisNames{k} '_Command/1'], ...
        [axisNames{k} '_Compare/1'],'autorouting','on');
    add_line(model,[axisNames{k} '_Command/1'], ...
        [axisNames{k} '_Lag/1'],'autorouting','on');
    add_line(model,[axisNames{k} '_Lag/1'], ...
        [axisNames{k} '_Compare/2'],'autorouting','on');
    add_line(model,[axisNames{k} '_Compare/1'], ...
        [axisNames{k} '_Scope/1'],'autorouting','on');
end
set_param(model,'SimulationCommand','update');
save_system(model,fullfile(outDir,[model '.slx']));
fprintf('Created outputs/%s.slx\n',model);
fprintf('Model shows commanded vs HYPOTHETICAL first-order response, not measurements.\n');
end
