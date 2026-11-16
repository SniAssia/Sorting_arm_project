function report = seven_fsm()
%SEVEN_FSM Document two finite-state machines and trace Arduino route states.
% No Stateflow dependency; uses base MATLAB tables, plots and route simulator.
% Vision FSM describes the Pi script. Arm FSM describes the Arduino sketch.
% Route traces represent nominal COMMAND execution, not physical sensor states.

rootDir=fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir,'src'));
folder=fullfile(rootDir,'outputs','fsm');
if ~exist(folder,'dir'),mkdir(folder);end

piRows={ ...
 'CAMERA_SETUP','camera started','CAPTURE','capture one frame; stop camera'; ...
 'CAPTURE','ROI extracted','CLASSIFY','calculate HSV pixel coverage'; ...
 'CLASSIFY','max coverage < 20 percent','NO_COLOR','send no Arduino command'; ...
 'CLASSIFY','RED/GREEN/BLUE reaches threshold','CHECK_ROUTE','open serial and send CHECK COLOR'; ...
 'CHECK_ROUTE','missing Route model check OK','REJECTED','do not command movement'; ...
 'CHECK_ROUTE','route check OK','ASK_CONFIRM','prompt operator yes/no'; ...
 'ASK_CONFIRM','response is not yes','DECLINED','do not command movement'; ...
 'ASK_CONFIRM','yes','SEND_ARM','send ARM; fixed one-second wait'; ...
 'SEND_ARM','wait expires','SEND_RUN','send RUN COLOR (no ARM ACK requirement)'; ...
 'SEND_RUN','command transmitted','MONITOR','monitor Arduino output'; ...
 'MONITOR','Color route complete line','DONE','report commanded completion'; ...
 'MONITOR','line begins ERR:','ERROR','report controller error'; ...
 'MONITOR','180 second deadline exceeded','TIMEOUT','print warning; arm may continue' ...
};
piTable=cell2table(piRows,'VariableNames',{'From','Trigger','To','Effect'});
writetable(piTable,fullfile(folder,'raspberry_pi_transitions.csv'));

arduinoRows={ ...
 'DISARMED','ARM','ARMED_IDLE','attach servos; command initial pose'; ...
 'ARMED_IDLE','RUN COLOR / route valid and gripper already open','MOVING','start waypoint 1'; ...
 'ARMED_IDLE','RUN COLOR / gripper must open first','OPENING','step gripper toward OPEN'; ...
 'OPENING','gripper interpolation complete','MOVING','start waypoint 1'; ...
 'MOVING','waypoint 2 PICK reached','GRIPPER_ACTION','close gripper'; ...
 'MOVING','waypoint 6 DROP reached','GRIPPER_ACTION','open gripper'; ...
 'MOVING','nonfinal waypoint reached without gripper action','WAITING','start one-second pause'; ...
 'GRIPPER_ACTION','gripper command finished','WAITING','start one-second pause'; ...
 'WAITING','automatic timer 1000 ms elapsed','MOVING','advance to next waypoint'; ...
 'WAITING','manual NEXT received (MANUAL route)','MOVING','advance to next waypoint'; ...
 'MOVING','waypoint 8 HOME completed','ARMED_IDLE','print Color route complete'; ...
 'MOVING','STOP','ARMED_IDLE','cancel interpolation, HOLD last servo command'; ...
 'WAITING','STOP','ARMED_IDLE','cancel route, HOLD last servo command'; ...
 'GRIPPER_ACTION','STOP','ARMED_IDLE','cancel route, HOLD last servo command'; ...
 'OPENING','STOP','ARMED_IDLE','cancel route, HOLD last servo command' ...
};
arduinoTable=cell2table(arduinoRows,'VariableNames',{'From','Trigger','To','Effect'});
writetable(arduinoTable,fullfile(folder,'arduino_transitions.csv'));

fsmDraw(piTable,fullfile(folder,'raspberry_pi_fsm.png'), ...
    'Raspberry Pi: single-frame decision FSM');
fsmDraw(arduinoTable,fullfile(folder,'arduino_fsm.png'), ...
    'Arduino: command and waypoint FSM');

cfg=seven_config();
traceTables=struct();
for c=1:numel(cfg.colors)
    color=cfg.colors{c};
    sim=seven_simulate_route(color,cfg);
    records={0,'ARMED_IDLE','RUN accepted',0,'HOME'};
    for w=1:8
        % startIdx(w) is the sample immediately before interpolation starts.
        startTime=sim.T(sim.startIdx(w));
        records(end+1,:)={startTime,'MOVING','Start waypoint',w,sim.routeNames{w}}; %#ok<AGROW>
        arrival=sim.arrivalTime(w);
        if w==2 || w==6
            records(end+1,:)={arrival,'GRIPPER_ACTION', ...
                sim.waypointActions{w},w,sim.routeNames{w}}; %#ok<AGROW>
            if w<8
                zone=sim.endIdx(w)+1:sim.startIdx(w+1);
            else
                zone=sim.endIdx(w)+1:numel(sim.T);
            end
            gripperIndices=zone(strcmp(sim.phases(zone),'GRIPPER'));
            assert(~isempty(gripperIndices),'Missing gripper action samples');
            lastTime=sim.T(gripperIndices(end));
        else
            lastTime=arrival;
        end
        if w<8
            records(end+1,:)={lastTime,'WAITING','Pause before next waypoint',w, ...
                sim.routeNames{w}}; %#ok<AGROW>
        else
            records(end+1,:)={lastTime,'ARMED_IDLE','Route command complete',8, ...
                'HOME'}; %#ok<AGROW>
        end
    end
    time=cell2mat(records(:,1));
    step=cell2mat(records(:,4));
    states=string(records(:,2));
    triggers=string(records(:,3));
    poses=string(records(:,5));
    assert(all(diff(time)>=-1e-7),'FSM trace went backwards in time');
    traceTable=table(time,states,triggers,step,poses, ...
        'VariableNames',{'Time_s','State','Event','Waypoint','Pose'});
    writetable(traceTable,fullfile(folder,['trace_' color '.csv']));
    traceTables.(color)=traceTable;
    names={'ARMED_IDLE','MOVING','GRIPPER_ACTION','WAITING'};
    value=zeros(numel(states),1);
    for k=1:numel(states)
        value(k)=find(strcmp(names,char(states(k))),1);
    end
    f=figure('Color','w','Visible','off','Name',['FSM ' color]);
    stairs(time,value,'-o','LineWidth',1.2,'MarkerSize',3);
    yticks(1:numel(names));yticklabels(names);ylim([0.5,numel(names)+0.5]);
    xlim([0,max(sim.T(end),1)]);grid on;
    xlabel('Nominal command time (s)'); ylabel('Controller state');
    title([color ': FSM route trace (no measured joint feedback)']);
    exportgraphics(f,fullfile(folder,['timeline_' color '.png']), ...
        'Resolution',170);close(f);
end
report.piTransitions=piTable;
report.arduinoTransitions=arduinoTable;
report.trace=traceTables;
fprintf('FSM tables, diagrams, and three command traces saved in MATLAB/outputs/fsm/\n');
end

function fsmDraw(transitions,path,figureTitle)
% GraphPlot is part of base MATLAB; full transition guards remain in CSV.
G=digraph(transitions.From,transitions.To);
f=figure('Color','w','Visible','off','Name',figureTitle, ...
    'Position',[100 100 1400 850]);
p=plot(G,'Layout','force','NodeFontSize',8,'MarkerSize',6,'ArrowSize',8);
p.LineWidth=1.15;
title({figureTitle,'Arrows indicate allowed transitions; triggers are in CSV'});
axis off;
exportgraphics(f,path,'Resolution',170);close(f);
end
