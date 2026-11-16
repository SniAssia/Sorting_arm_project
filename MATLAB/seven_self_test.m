function seven_self_test()
%SEVEN_SELF_TEST Verify core mathematical consistency and route constraints.
rootDir=fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir,'src'));
cfg=seven_config();

% Verify all stored model waypoints can be solved by IK and round-trip FK.
for k=1:numel(cfg.poseNames)
    xyz=cfg.poseXYZ(k,:);
    q=seven_ik(xyz,cfg);
    assert(all(q>=0)&all(q<=180),'Joint bounds violated');
    assert(norm(seven_fk(q,cfg)-xyz)<1e-4,'FK/IK mismatch');
end

% Arduino's initial command matches model HOME up to pose rounding.
assert(norm(seven_fk(cfg.initialQ,cfg)-cfg.poseXYZ(1,:))<0.05, ...
    'HOME pose differs substantially from initial arm joint commands');

% Test multiple attainable configurations with mirrored base angles.
for b=[15 65 115 165]
    q0=[b 75 100];
    xyz=seven_fk(q0,cfg);
    q1=seven_ik(xyz,cfg);
    assert(norm(q0-q1)<1e-6,'Kinematic round-trip failed');
end

% Check that unreachable poses are rejected.
failed=false;
try
    seven_ik([50 0 0],cfg);
catch
    failed=true;
end
assert(failed,'A clearly unreachable target was not rejected');

for k=1:numel(cfg.colors)
    sim=seven_simulate_route(cfg.colors{k},cfg);
    assert(numel(sim.routeIds)==8,'Wrong number of waypoints');
    assert(all(diff(sim.T)>0),'Time should strictly increase');
    assert(all(max(abs(diff(sim.Q)),[],2)<=cfg.jointStepDeg+1e-7), ...
        'Joint command exceeds 1 deg per step');
    assert(all(abs(diff(sim.G))<=1),'Gripper moves too far in one command step');
    assert(sim.G(1)==cfg.gripOpen && sim.G(end)==cfg.gripOpen, ...
        'Gripper should begin and end OPEN');
    assert(all(sim.Q(:)>=-1e-6) && all(sim.Q(:)<=180+1e-6), ...
        'Joint command violates software range');
    for w=1:8
        idx=sim.endIdx(w);
        assert(norm(sim.XYZ(idx,:)-sim.routeXYZ(w,:))<1e-4, ...
            'Waypoint endpoint mismatch');
    end
    fprintf('%s PASS: route %.1f s, pickup drift %.2f cm\n', ...
        sim.color,sim.durationSeconds,sim.maxPickupDriftCm);
end
fprintf('PASS: all FK/IK, route, timing, and gripper checks.\n');
end
