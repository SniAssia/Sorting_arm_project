%% SEVEN robotic arm: kinematic modeling and joint-command routes
% Runs in base MATLAB, without Control System Toolbox / Robotics Toolbox.
% All XYZ/lengths are from team's Arduino sketch until physically calibrated.
clear; clc; close all;
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir,'src'));
outDir = fullfile(rootDir,'outputs');
if ~exist(outDir,'dir'), mkdir(outDir); end

cfg = seven_config();

%% 1. Mathematical check: XYZ -> IK -> FK for every Arduino pose
n = numel(cfg.poseNames);
Qposes = zeros(n,3);
reconstructionErrorCm = zeros(n,1);
for i=1:n
    Qposes(i,:) = seven_ik(cfg.poseXYZ(i,:),cfg);
    reconstructionErrorCm(i) = norm(seven_fk(Qposes(i,:),cfg)-cfg.poseXYZ(i,:));
end
poseTable = table(cfg.poseNames(:),cfg.poseXYZ(:,1),cfg.poseXYZ(:,2), ...
    cfg.poseXYZ(:,3),Qposes(:,1),Qposes(:,2),Qposes(:,3), ...
    reconstructionErrorCm,'VariableNames', ...
    {'Pose','X_cm','Y_cm','Z_cm','Base_deg','Shoulder_deg', ...
     'Elbow_deg','FK_IK_error_cm'});
disp('=== XYZ -> IK -> FK validation (numerical consistency only) ===');
disp(poseTable);
writetable(poseTable,fullfile(outDir,'pose_validation.csv'));

%% 2. Simulate all routes and produce route-specific figures and tables
allRoutes = struct();
summaryDuration = zeros(numel(cfg.colors),1);
summaryPick = zeros(numel(cfg.colors),1);
summaryDrop = zeros(numel(cfg.colors),1);
summaryLag = zeros(numel(cfg.colors),1);
for c=1:numel(cfg.colors)
    color = cfg.colors{c};
    sim = seven_simulate_route(color,cfg);
    allRoutes.(color) = sim;
    colorDir = fullfile(outDir,color);
    if ~exist(colorDir,'dir'),mkdir(colorDir);end

    waypointTable = table((1:8)',sim.routeNames(:),sim.routeXYZ(:,1), ...
        sim.routeXYZ(:,2),sim.routeXYZ(:,3),sim.targetAngles(:,1), ...
        sim.targetAngles(:,2),sim.targetAngles(:,3),sim.arrivalTime(:), ...
        sim.waypointActions(:), 'VariableNames', ...
        {'Step','Pose','X_cm','Y_cm','Z_cm','Base_deg','Shoulder_deg', ...
         'Elbow_deg','Arrive_s','GripperAction'});
    writetable(waypointTable,fullfile(colorDir,'waypoints.csv'));

    commandTable = table(sim.T,sim.Q(:,1),sim.Q(:,2),sim.Q(:,3),sim.G, ...
        sim.XYZ(:,1),sim.XYZ(:,2),sim.XYZ(:,3),sim.phases(:), ...
        'VariableNames',{'Time_s','Base_deg','Shoulder_deg','Elbow_deg', ...
        'Gripper_deg','X_cm','Y_cm','Z_cm','Phase'});
    writetable(commandTable,fullfile(colorDir,'commands.csv'));
    seven_plot_results(sim,cfg,colorDir);
    summaryDuration(c) = sim.durationSeconds;
    summaryPick(c) = sim.maxPickupDriftCm;
    summaryDrop(c) = sim.maxDropDriftCm;
    summaryLag(c) = sim.maxAssumedLagCm;
end
summaryTable = table(cfg.colors(:),summaryDuration,summaryPick,summaryDrop,summaryLag, ...
    'VariableNames',{'Color','CommandDuration_s','MaxPickXYDrift_cm', ...
     'MaxDropXYDrift_cm','MaxHypotheticalLag_cm'});
writetable(summaryTable,fullfile(outDir,'route_summary.csv'));
disp('=== Nominal commanded route timing (NOT measured runtime) ===');
disp(summaryTable);

%% 3. MATLAB is intentionally used only for numerical analysis and 2D plots.
% Use PyBullet/ for the articulated 3D robot visualization.

%% 4. Simulink optional input from actual command trajectories (not image data)
red = allRoutes.RED;
seven_simulink_time = red.T;
seven_simulink_Q = red.Q;
seven_simulink_tau = cfg.assumedTau;
save(fullfile(outDir,'seven_simulink_inputs.mat'), ...
    'seven_simulink_time','seven_simulink_Q','seven_simulink_tau');
save(fullfile(outDir,'simulation_data.mat'),'allRoutes','cfg','poseTable','summaryTable');

%% 5. A concise text summary for the report
fid = fopen(fullfile(outDir,'summary.txt'),'w');
assert(fid>0,'Cannot create outputs/summary.txt');
fprintf(fid,'SEVEN MATLAB KINEMATIC MODEL - SIMULATION SUMMARY\n');
fprintf(fid,'L1 = %.2f cm | L2 = %.2f cm | initial q = [%.1f %.1f %.1f] deg\n', ...
    cfg.L1,cfg.L2,cfg.initialQ);
fprintf(fid,'Max numerical FK/IK round-trip error: %.8f cm\n', ...
    max(reconstructionErrorCm));
for c=1:numel(cfg.colors)
    sim = allRoutes.(cfg.colors{c});
    fprintf(fid,'%s: %.2f s nominal; pick drift %.3f cm; drop drift %.3f cm\n', ...
        sim.color,sim.durationSeconds,sim.maxPickupDriftCm,sim.maxDropDriftCm);
end
fprintf(fid,'IMPORTANT: all motion is predicted from COMMANDS; no measured joint feedback.\n');
fprintf(fid,'Dynamics uses illustrative tau %.2f s; not experimentally identified.\n', ...
    cfg.assumedTau);
fprintf(fid,'Limits are software/conservative reach constraints, NOT collision checks.\n');
fclose(fid);

fprintf('\nDONE. Open outputs/ for CSV, PNG and MAT files.\n');
fprintf('For optional Simulink model, type: seven_build_simulink\n');
fprintf('For independent numerical checks in MATLAB: seven_self_test\n');
fprintf('For MATLAB finite-state-machine documentation: seven_fsm\n');
