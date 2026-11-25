function model = seven_build_simulink()
%SEVEN_BUILD_SIMULINK Replay the four red-route commands from the bundled CSV.
rootDir = fileparts(mfilename('fullpath'));
route = readtable(fullfile(rootDir,'..','PyBullet','trajectories','RED_commands.csv'));
commands = route{:,{'Base_deg','Shoulder_deg','Elbow_deg','Gripper_deg'}};
outDir = fullfile(rootDir,'outputs');
if ~isfolder(outDir), mkdir(outDir); end

model = 'SEVEN_Command_Replay';
if bdIsLoaded(model), close_system(model,0); end
new_system(model);
workspace = get_param(model,'ModelWorkspace');
% The firmware timers share a 10 ms grid. Align CSV timestamps to solver ticks.
step = 0.01;
time = round(route.Time_s / step) * step;
assignin(workspace,'routeCommands',[time commands]);
set_param(model,'Solver','FixedStepDiscrete','FixedStep',num2str(step), ...
    'StopTime',num2str(time(end)));
add_block('simulink/Sources/From Workspace',[model '/Route commands'], ...
    'VariableName','routeCommands','SampleTime',num2str(step),'Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value','Position',[60 75 200 115]);
add_block('simulink/Sinks/Scope',[model '/Command angles'], ...
    'Position',[350 40 400 85]);
add_block('simulink/Sinks/Out1',[model '/JointCommands_deg'], ...
    'Position',[350 130 400 150]);
add_line(model,'Route commands/1','Command angles/1','autorouting','on');
add_line(model,'Route commands/1','JointCommands_deg/1','autorouting','on');
save_system(model,fullfile(outDir,[model '.slx']));
end
