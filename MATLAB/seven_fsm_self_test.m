function seven_fsm_self_test()
%SEVEN_FSM_SELF_TEST Check MATLAB-generated FSM tables/command traces.
rootDir=fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir,'src'));
report=seven_fsm();
cfg=seven_config();
assert(height(report.piTransitions)==13,'Pi decision FSM unexpected size');
assert(height(report.arduinoTransitions)>=12,'Arduino FSM unexpectedly small');
for c=1:numel(cfg.colors)
    color=cfg.colors{c};
    trace=report.trace.(color);
    sim=seven_simulate_route(color,cfg);
    assert(all(diff(trace.Time_s)>=-1e-9),'Nonmonotonic event log');
    assert(sum(trace.State=="MOVING")==8,'Expected eight waypoint moves');
    assert(sum(trace.State=="GRIPPER_ACTION")==2,'Expected close and open');
    assert(sum(trace.State=="WAITING")==7,'Expected seven waypoint pauses');
    assert(trace.State(end)=="ARMED_IDLE",'Final controller state must be idle');
    assert(abs(trace.Time_s(end)-sim.T(end))<1e-8,'Trace end time mismatch');
    assert(any(trace.Event=="CLOSE") && any(trace.Event=="OPEN"), ...
        'Pick/drop gripper commands missing');
    fprintf('FSM %s PASS: 8 moves, 2 grip actions, 7 pauses, %.1f s\n', ...
        color,sim.T(end));
end
fprintf('PASS: MATLAB FSM specifications and traces validated.\n');
end
