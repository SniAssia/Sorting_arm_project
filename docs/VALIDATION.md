# Verification plan and evidence

The baseline uses one firmware route per color. The enhanced controller checks a persistent count before `CHECK COLOR`, waits for an `ARM` acknowledgement, and updates the count after successful sequence completion. Each bin accepts three objects at its original fixed destination. [Bin capacity FSM](SLOT_TRACKING_AND_FSM.md)


## 1. Tests included in the package

| Test | Command | What it establishes |
|---|---|---|
| Source integrity and package layout | `py tools/preflight.py` | Original-source fingerprints, routes, URDF, documentation presence |
| Exported routes and FK | `py PyBullet/simulate.py --verify-only --all` | Valid time series, gripper events, Python FK consistent with command CSV |
| URDF transform-chain | from `PyBullet/`: `py test_urdf.py` | Generic arm joint-frame math agrees with the Arduino FK |
| MATLAB kinematics | `seven_self_test` | Nine configured poses and three 8-step routes pass model assertions |
| MATLAB finite state machines | `seven_fsm_self_test` | FSM route traces have 8 moves, 2 gripper events, 7 waits |
| Bin capacity controller tests | `python -m unittest discover -s tests -v` | Packaged fixed routes, FULL rejection, persistence, timeout, duplicate completion, reset and interprocess exclusion with mocked serial |
| Capacity FSM | `seven_slot_fsm`; `seven_slot_fsm_self_test` | Abstract state transitions, PENDING reconciliation and 3-per-color invariant |
| Simulink | `seven_build_simulink`; run generated model | Scopes compare commands and first-order *assumed* response |
| PyBullet GUI | `py simulate.py --color RED` (and other colors) | Visualizes generic robot and scripted pick-and-place route |

`--verify-only` does not require installed PyBullet. The GUI and full Simulink/Matlab functionality require those programs on your computer. As with any software package, repeat the checks after changing source or geometry.

## 2. Numerical reference from existing runs

| Quantity | RED | GREEN | BLUE |
|---|---:|---:|---:|
| CSV samples | 430 | 404 | 468 |
| Nominal command duration (s) | 45.7 | 43.1 | 49.5 |
| Scripted close/grip complete (s) | 14.8 | 14.8 | 14.8 |
| Scripted release complete (s) | 38.1 | 36.8 | 40.0 |
| Max modeled pickup XY drift (cm) | 2.7313 | 2.7313 | 2.7313 |

Nine configured target coordinates had numerical IK→FK reconstruction errors on the order of floating-point roundoff. In the user's previous local PyBullet simulation, tool-point mismatch was approximately `0.00000099 cm`; **this merely checks model-frame consistency**. Simulink screenshots supplied by the team are preserved under `docs/evidence/` but show a **hypothetical lag**, not measured servo signals.

## 3. Physical testing to complete before final submission

Use `docs/physical_test_template.csv`. Recommended repeated test cases: red, green, blue under consistent lighting; a no-color/UNKNOWN case; a moved object outside the fixed ROI; recovery to HOME; and servo power stability during grasp. Record the *observed* destination bin, missed grasps, timing, and any mechanical contact. Keep numerical command durations in a **separate column** from measured physical cycle times. Do not claim a success rate unless you have counts of observed successes and total trials.

Known model risks: no physical joint encoders, no collision detection, no verified force-controlled grasp, software 0–180° range not a true mechanical safety range, and up to ~2.73 cm modeled sideways movement during pickup descent.


## Capacity extension validation boundary

Mocked serial tests exercise the production CLI and inspect the persistent counter separately from the physical outcome. On hardware, log the observed deposit alongside the completion message, compare estimated and observed occupancy after each of three runs to the same waypoint, then verify fourth-object rejection and confirmed reset after emptying the bin.
