function seven_plot_results(sim,cfg,outDir)
%SEVEN_PLOT_RESULTS Export only 2D model analyses (no MATLAB 3D render).
% These are mathematical predictions based on commands, not sensor data.

% Positioning and gripper setpoints share the same command clock.
f=figure('Name',[sim.color ' Joint Commands'],'Color','w','Visible','off');
stairs(sim.T,[sim.Q sim.G],'LineWidth',1.4);
grid on; xlabel('Command time (s)'); ylabel('Command angle (deg)');
legend({'Base','Shoulder','Elbow','Gripper'},'Location','best');
title([sim.color ': joint and gripper commands']);
exportgraphics(f,fullfile(outDir,'joint_angles.png'),'Resolution',170);
close(f);

% Gripper servo angle.
f=figure('Name',[sim.color ' Gripper'],'Color','w','Visible','off');
plot(sim.T,sim.G,'LineWidth',1.8);
hold on; yline(cfg.gripOpen,':','OPEN'); yline(cfg.gripClosed,':','CLOSED');
ylim([cfg.gripOpen-8,cfg.gripClosed+8]);
xlabel('Time (s)'); ylabel('Commanded gripper angle (deg)');
title([sim.color ': grip after PICK, release after DROP']); grid on;
exportgraphics(f,fullfile(outDir,'gripper_commands.png'),'Resolution',170);
close(f);

% XY projection can show lateral drift without a MATLAB 3D simulation.
f=figure('Name',[sim.color ' XY Footprint'],'Color','w','Visible','off');
plot(sim.XYZ(:,1),sim.XYZ(:,2),'LineWidth',1.65); hold on;
scatter(sim.routeXYZ(:,1),sim.routeXYZ(:,2),32,'k','filled');
for k=1:numel(sim.routeNames)
    text(sim.routeXYZ(k,1)+0.3,sim.routeXYZ(k,2)+0.3, ...
        sprintf('%d %s',k,sim.routeNames{k}),'FontSize',8);
end
xlabel('X (cm)'); ylabel('Y (cm)'); axis equal; grid on;
title([sim.color ': predicted XY projection of end-effector path']);
legend({'Commanded XY','Target waypoints'},'Location','best');
exportgraphics(f,fullfile(outDir,'xy_footprint.png'),'Resolution',170);
close(f);

% End-effector height versus time.
f=figure('Name',[sim.color ' Height'],'Color','w','Visible','off');
plot(sim.T,sim.XYZ(:,3),'LineWidth',1.6); grid on;
xlabel('Time (s)');ylabel('Predicted Z (cm)');
title([sim.color ': end-effector height from forward kinematics']);
exportgraphics(f,fullfile(outDir,'z_over_time.png'),'Resolution',170);
close(f);
end
