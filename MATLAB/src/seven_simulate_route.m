function sim = seven_simulate_route(color, cfg)
%SEVEN_SIMULATE_ROUTE Reproduces Arduino's discrete joint/grip command ramp.
% Does not prove that the physical servos reach the commanded positions.
color = upper(char(color));
if ~ismember(color,cfg.colors)
    error('seven_simulate_route:color','Expected RED, GREEN or BLUE.');
end
routeIds = cfg.(color);
nWaypoints = numel(routeIds);
qTarget = zeros(nWaypoints,3);
for w = 1:nWaypoints
    qTarget(w,:) = seven_ik(cfg.poseXYZ(routeIds(w),:),cfg);
end

t = 0;
q = cfg.initialQ;
grip = cfg.gripOpen;
T = 0;
Q = q;
G = grip;
phases = {'START'};
startIdx = zeros(nWaypoints,1);
endIdx = zeros(nWaypoints,1);
arrivalTime = zeros(nWaypoints,1);
waypointActions = repmat({'NONE'},nWaypoints,1);

for w = 1:nWaypoints
    startIdx(w) = numel(T);
    target = qTarget(w,:);
    while max(abs(target - q)) > 1e-8
        delta = target - q;
        q = q + sign(delta).*min(abs(delta),cfg.jointStepDeg);
        t = t + cfg.jointDt;
        T(end+1,1) = t; %#ok<AGROW>
        Q(end+1,:) = q; %#ok<AGROW>
        G(end+1,1) = grip; %#ok<AGROW>
        phases{end+1,1} = 'JOINT_MOVE'; %#ok<AGROW>
    end
    endIdx(w) = numel(T);
    arrivalTime(w) = t;

    % In Arduino: close after PICK (step 2); open after DROP (step 6).
    if w == 2
        targetGrip = cfg.gripClosed;
        waypointActions{w} = 'CLOSE';
    elseif w == 6
        targetGrip = cfg.gripOpen;
        waypointActions{w} = 'OPEN';
    else
        targetGrip = grip;
    end
    while grip ~= targetGrip
        grip = grip + sign(targetGrip-grip); % exactly 1 degree per 30 ms
        t = t + cfg.gripDt;
        T(end+1,1) = t; %#ok<AGROW>
        Q(end+1,:) = q; %#ok<AGROW>
        G(end+1,1) = grip; %#ok<AGROW>
        phases{end+1,1} = 'GRIPPER'; %#ok<AGROW>
    end
    % Arduino awaits 1000 ms before auto-advancing each non-final step.
    if w < nWaypoints
        t = t + cfg.pauseDt;
        T(end+1,1) = t; %#ok<AGROW>
        Q(end+1,:) = q; %#ok<AGROW>
        G(end+1,1) = grip; %#ok<AGROW>
        phases{end+1,1} = 'WAIT'; %#ok<AGROW>
    end
end

XYZ = seven_fk(Q,cfg);

% Illustrative motor lag. This is explicitly NOT hardware feedback.
Qlag = zeros(size(Q));
Qlag(1,:) = Q(1,:);
for k = 2:numel(T)
    dt = T(k)-T(k-1);
    alpha = 1 - exp(-dt/cfg.assumedTau);
    Qlag(k,:) = Qlag(k-1,:) + alpha*(Q(k,:)-Qlag(k-1,:));
end
XYZlag = seven_fk(Qlag,cfg);

% Nominal vertical descent versus actual arc under joint-space interpolation.
idxPick = startIdx(2):endIdx(2);
idxDrop = startIdx(6):endIdx(6);
xyPickTarget = cfg.poseXYZ(routeIds(2),1:2);
xyDropTarget = cfg.poseXYZ(routeIds(6),1:2);
pickDrift = hypot(XYZ(idxPick,1)-xyPickTarget(1),XYZ(idxPick,2)-xyPickTarget(2));
dropDrift = hypot(XYZ(idxDrop,1)-xyDropTarget(1),XYZ(idxDrop,2)-xyDropTarget(2));

sim.color = color;
sim.routeIds = routeIds;
sim.routeNames = cfg.poseNames(routeIds);
sim.routeXYZ = cfg.poseXYZ(routeIds,:);
sim.targetAngles = qTarget;
sim.T = T;
sim.Q = Q;
sim.G = G;
sim.phases = phases;
sim.XYZ = XYZ;
sim.Qlag = Qlag;
sim.XYZlag = XYZlag;
sim.startIdx = startIdx;
sim.endIdx = endIdx;
sim.arrivalTime = arrivalTime;
sim.waypointActions = waypointActions;
sim.durationSeconds = T(end);
sim.maxPickupDriftCm = max(pickDrift);
sim.maxDropDriftCm = max(dropDrift);
sim.maxAssumedLagCm = max(sqrt(sum((XYZ-XYZlag).^2,2)));
end
