function xyz = seven_fk(q, cfg)
%SEVEN_FK Forward kinematics for 3-DOF SEVEN model (shoulder-centered).
% q: N-by-3 [base shoulder elbow] in degrees. xyz: N-by-3 in cm.
if size(q,2) ~= 3
    error('seven_fk:shape','Joint angles must be an N-by-3 array.');
end
q = double(q);
b = q(:,1) * (pi/180);
s = q(:,2) * (pi/180);
e = q(:,3) * (pi/180);
h = cfg.L1 .* cos(s) + cfg.L2 .* cos(s + e - pi);
z = cfg.L1 .* sin(s) + cfg.L2 .* sin(s + e - pi);
xyz = [h .* sin(b), h .* cos(b), z];
end
