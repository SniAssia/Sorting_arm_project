function seven_slot_fsm_self_test()
%SEVEN_SLOT_FSM_SELF_TEST Base MATLAB logical assertions, not hardware test.
r=seven_slot_fsm();
assert(isequal(r.testCounts,[3 3 3]));
T=r.transitions;
assert(any(strcmp(T.From,'READY') & strcmp(T.To,'COLOR_DETECTED')));
assert(any(strcmp(T.From,'COLOR_DETECTED') & strcmp(T.To,'READY')));
assert(any(strcmp(T.From,'COLOR_DETECTED') & strcmp(T.To,'CAPACITY_CHECK')));
assert(any(strcmp(T.From,'CAPACITY_CHECK') & strcmp(T.To,'FULL')));
assert(any(strcmp(T.From,'FULL') & strcmp(T.To,'READY')));
assert(any(strcmp(T.From,'CAPACITY_CHECK') & strcmp(T.To,'ROUTE_VALIDATION')));
assert(any(strcmp(T.From,'ROUTE_VALIDATION') & strcmp(T.To,'READY')));
assert(any(strcmp(T.From,'ROUTE_VALIDATION') & strcmp(T.To,'CONFIRMATION')));
assert(any(strcmp(T.From,'CONFIRMATION') & strcmp(T.To,'READY')));
assert(any(strcmp(T.From,'CONFIRMATION') & strcmp(T.To,'EXECUTION')));
assert(any(strcmp(T.From,'EXECUTION') & strcmp(T.To,'COMPLETION')));
assert(any(strcmp(T.From,'EXECUTION') & strcmp(T.To,'PENDING_REVIEW')));
assert(any(strcmp(T.From,'COMPLETION') & strcmp(T.To,'COUNTER_UPDATE')));
assert(any(strcmp(T.From,'COMPLETION') & strcmp(T.To,'PENDING_REVIEW')));
assert(any(strcmp(T.From,'PENDING_REVIEW') & strcmp(T.To,'COUNTER_UPDATE')));
assert(any(strcmp(T.From,'PENDING_REVIEW') & strcmp(T.To,'READY')));
assert(any(strcmp(T.From,'COUNTER_UPDATE') & strcmp(T.To,'READY')));
assert(any(strcmp(T.From,'READY') & strcmp(T.To,'MANUAL_RESET')));
assert(any(strcmp(T.From,'MANUAL_RESET') & strcmp(T.To,'READY')));
fprintf('PASS: bin capacity, completion, recovery and reset transitions.\n');
end
