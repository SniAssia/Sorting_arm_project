function cfg = seven_config()
%SEVEN_CONFIG Robot geometry and routing read from team's Arduino sketch.
% Units: cm / degrees / seconds. These are modeled inputs, not calibration data.
cfg.L1 = 12.0;
cfg.L2 = 12.0;
cfg.initialQ = [105, 73, 60];
cfg.gripOpen = 90;
cfg.gripClosed = 115;
cfg.jointStepDeg = 1.0;
cfg.jointDt = 0.100;
cfg.gripDt = 0.030;
cfg.pauseDt = 1.000;
cfg.assumedTau = 0.35; % HYPOTHETICAL servo time constant, not measured

% Names and XYZ copied from custom Arduino's poses[POSE_COUNT].
cfg.poseNames = {'HOME','AP','PICK','AR','DR','AG','DG','AB','DB'};
cfg.poseXYZ = [
 11.29,   -3.03,     2.70;    % HOME - rounded FK(105,73,60)
  3.93,   14.68,    14.18;    % AP - approach to pickup
  3.93,   14.68,    -4.00;    % PICK
 13.14,    6.13,     8.00;    % AR - above red bin
 13.14,    6.13,     2.00;    % DR - drop red
 13.14,   -6.13,     8.00;    % AG - above green bin
 13.14,   -6.13,     2.00;    % DG - drop green
  3.7528,-14.0055,   8.00;    % AB - above blue bin
  3.7528,-14.0055,   2.00     % DB - drop blue
];

% Index mapping: HOME=1 AP=2 PICK=3 AR=4 DR=5 AG=6 DG=7 AB=8 DB=9.
cfg.RED   = [2,3,2,1,4,5,4,1];
cfg.GREEN = [2,3,2,1,6,7,6,1];
cfg.BLUE  = [2,3,2,1,8,9,8,1];
cfg.colors = {'RED','GREEN','BLUE'};

% Conservative workspace checks from Arduino, not a geometric safety proof.
cfg.minX = 1.0;
cfg.minZ = -6.0;
cfg.minD = 3.0;
cfg.maxD = 23.0;
cfg.jointLower = [0,0,0];
cfg.jointUpper = [180,180,180];
cfg.roundtripToleranceCm = 0.1;
end
