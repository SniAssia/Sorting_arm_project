function seven_slot_fsm_self_test()
%SEVEN_SLOT_FSM_SELF_TEST Base MATLAB logical assertions, not hardware test.
r=seven_slot_fsm();
assert(isequal(r.testCounts,[3 3 3]));
T=r.transitions;
assert(any(strcmp(T.From,'READY') & strcmp(T.To,'FULL')));
assert(any(strcmp(T.From,'READY') & strcmp(T.To,'UNCALIBRATED')));
assert(any(strcmp(T.From,'CONFIRM') & strcmp(T.To,'PENDING')));
assert(any(strcmp(T.From,'PENDING_REVIEW') & strcmp(T.To,'CANCELLED')));
assert(any(strcmp(T.From,'VERIFY_PLACEMENT') & strcmp(T.To,'COMMITTED')));
assert(numel(unique(T.From))>=10);
fprintf('PASS: 3 slots per color, FULL, PENDING, and reconciliation FSM rules.\n');
end
