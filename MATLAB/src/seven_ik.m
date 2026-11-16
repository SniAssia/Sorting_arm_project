function q = seven_ik(xyz, cfg)
%SEVEN_IK Exact same IK *branch* and conservative checks as Arduino sketch.
% xyz [x y z] in cm, q [base shoulder elbow] in degrees.
if ~isnumeric(xyz) || numel(xyz) ~= 3
    error('seven_ik:shape','XYZ must contain three numeric coordinates.');
end
xyz = reshape(double(xyz),1,3);
x = xyz(1); y = xyz(2); z = xyz(3);
if any(~isfinite(xyz)) || x < cfg.minX || z < cfg.minZ
    error('seven_ik:workspace','XYZ violates Arduino minX/minZ checks.');
end
r = hypot(x,y);
d = hypot(r,z);
if d < cfg.minD || d > cfg.maxD
    error('seven_ik:distance','XYZ violates Arduino 3..23 cm radial bounds.');
end
cE = (cfg.L1^2 + cfg.L2^2 - d^2)/(2*cfg.L1*cfg.L2);
cA = (cfg.L1^2 + d^2 - cfg.L2^2)/(2*cfg.L1*d);
clamp = @(v) max(-1.0, min(1.0, v));
b = atan2(x,y);
s = atan2(z,r) + acos(clamp(cA));
e = acos(clamp(cE));
q = [b s e] * (180/pi);
if any(~isfinite(q)) || any(q < cfg.jointLower - 1e-8) || ...
        any(q > cfg.jointUpper + 1e-8) || ...
        norm(seven_fk(q,cfg)-xyz) >= cfg.roundtripToleranceCm
    error('seven_ik:limits','Target violates joint limits or FK/IK accuracy check.');
end
end
