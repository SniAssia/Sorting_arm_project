# SEVEN | Vision-Guided Robotic Color Sorting

**Embedded Systems | UM6P College of Computing**  
**Group members:** Assia Snissi, Khadija Lekbiri, Manal Zaidi, Rhita Rhallami 

**System status:** Working color-sorting MVP reported on physical hardware; validated baseline mathematical and PyBullet command replay; bin capacity tracking implemented in Raspberry Pi software, pending physical trials.

## Project summary

This project demonstrates an **MVP of an industrial color-sorting system** using a 3D-printed SEVEN arm, an Arduino Uno, a Raspberry Pi camera, and Python/OpenCV. The camera classifies a single object as red, green, blue, or unknown. The Arduino moves the arm through eight programmed waypoints to pick up and route the object to its assigned bin. The upgraded Raspberry Pi controller additionally tracks **three objects per color bin** and retains independent occupancy estimates across restarts.

The SEVEN physical platform is adapted from [T. Selçuk Yüksel's open-source robotic arm](https://github.com/elevenMiles/Robotic_Arm_Seven), under the repository's MIT license. The submitted Arduino firmware is the team's custom control program, not the upstream sketch.

All objects of a given color use the same calibrated firmware waypoint. The Pi checks capacity before opening the serial port. Once the count reaches three, it reports the bin as full.

## 1. Scope and architecture

```text
Raspberry Pi camera (640×480)
    → 200×200-pixel ROI + HSV color coverage (>=20%)
    → check the detected color’s bin count (0..3)
    → Arduino CHECK route + operator approval
    → persist PENDING transaction → ARM → RUN
    → joint-space pick-and-place sequence
    → Arduino reports sequence completion → persist count +1 and clear PENDING
```

Three positioning joints control the base, shoulder and elbow; a fourth SG90 servo operates the gripper. The nominal arm model uses two 12 cm links. The sorting controller is **open-loop**: it has no position encoders, object-contact sensor, collision monitor, or automatic confirmation of physical deposit.

The three bins use RED → DR, GREEN → DG and BLUE → DB. UNKNOWN produces no route. Sorting runs once per invocation.

## 2. Directory guide

| Location | Purpose |
|---|---|
| `Original_Source_Code/Arduino/SEVEN_color_sorter_original/*.ino` | physical robot firmware |
| `Original_Source_Code/RaspberryPi/color_sorter_original.py` | single-drop Pi implementation (faithful earlier transcription) |
| `Original_Source_Code/RaspberryPi/sort_with_slots.py` | persistent bin capacity controller |
| `Original_Source_Code/SOURCE_MANIFEST.json` | Source provenance and original-file SHA-256 hashes |
| `config/slots.json` | Capacity and firmware destination identifier for each color |
| `MATLAB/` | FK/IK, route modeling, plots, numerical tests and state machines |
| `MATLAB/outputs/{RED,GREEN,BLUE}/commands.csv` | Verified fixed-destination command trajectories |
| `MATLAB/seven_slot_fsm.m` | Capacity-aware controller FSM and logical example |
| `PyBullet/` | Generic 3D arm, baseline sorting replay, URDF and verification |
| `tests/` | Python unit tests using a simulated Arduino serial interface |
| `tools/preflight.py` | Offline package/source-integrity checks |
| `docs/` | Architecture, FSM specifications, testing, evidence checklist |
| `runtime/` | Local occupancy/camera output created on execution; intentionally not tracked |


## 3. Reproduce the numerical work (Windows/Linux; no hardware required)

Prerequisites: Python 3 and MATLAB. Simulink is only required for the servo-tracking block model. PyBullet is only required for the 3D viewer.

From the repository root:

```powershell
python tools/preflight.py
python -m unittest discover -s tests -v
python PyBullet/simulate.py --verify-only --all
python PyBullet/test_urdf.py
```

Windows users can alternatively run:

```powershell
powershell -ExecutionPolicy Bypass -File .\RUN_CHECKS.ps1
```

The new controller can be inspected without a camera, an Arduino, or writing an occupancy file:

```powershell
python Original_Source_Code/RaspberryPi/sort_with_slots.py --status
python Original_Source_Code/RaspberryPi/sort_with_slots.py --verify-config
python Original_Source_Code/RaspberryPi/sort_with_slots.py --color RED --dry-run
```

The offline tests exercise the packaged bin configuration and the controller CLI with mocked serial communication. They cover capacity limits, fixed routes, persistence, cancellation, timeout, concurrent access, reset and recovery. Hardware-free previews leave occupancy unchanged.

### MATLAB and Simulink

Open MATLAB and set the Current Folder to the repository's `MATLAB` directory:

```matlab
run_SEVEN_project         % FK/IK, command trajectories, 2D plots and CSV
seven_self_test           % original 8-waypoint numerical checks
seven_fsm                 % original Pi/Arduino FSM transitions and traces
seven_fsm_self_test
seven_slot_fsm            % bin capacity FSM and abstract capacity trace
seven_slot_fsm_self_test
seven_build_simulink      % generates outputs/SEVEN_Servo_Tracking.slx
```

Open the generated Simulink model and run its Base, Shoulder and Elbow scopes. Its first-order actuator response (`tau = 0.35 s`) is **an illustrative assumption**, not a measured SG90 motor model. The MATLAB/Simulink project is numerical and diagrammatic; it does not contain a 3D renderer.

### PyBullet: generic 3D arm

```powershell
python -m pip install -r PyBullet/requirements.txt
python PyBullet/simulate.py --color RED
python PyBullet/simulate.py --color GREEN
python PyBullet/simulate.py --color BLUE --screenshot
python PyBullet/simulate.py --all --headless
```

PyBullet replays the exported firmware routes through a generic articulated URDF. The simulation scripts object grasp and release. Each replay uses the same destination as the enhanced controller for that color.

## 4. Physical robot setup and baseline reproduction

**Hardware:** SEVEN 3D-printed arm, Arduino Uno, four servo motors, Raspberry Pi and camera, external suitable servo supply, USB serial connection and common electrical ground. The original firmware maps base/shoulder/elbow/gripper to Arduino `D6/D9/D10/D11` respectively. Do **not** power four servos from the Arduino 5 V output.

1. With servo power controlled and mechanical clearances checked, upload `Original_Source_Code/Arduino/SEVEN_color_sorter_original/SEVEN_color_sorter_original.ino` via Arduino IDE (board/port per actual setup).
2. Use Arduino Serial Monitor at **115200 baud**, newline termination. Check `HELP`, `POSES` and `CHECK RED/GREEN/BLUE` before motion.
3. Confirm the physical HOME, pickup and bin positions, servo orientation, grip open/closed calibration, and a safe startup posture. `ARM` attaches servos and commands a stored initial pose.
4. On the Raspberry Pi, use an OS environment supporting Picamera2, OpenCV, NumPy and pyserial. The untouched baseline script is `Original_Source_Code/RaspberryPi/color_sorter_original.py`. It captures **one frame** and asks for operator confirmation before `RUN COLOR`.
5. The camera ROI is `x=220:420, y=140:340` and the color threshold is 20% HSV-mask **pixel coverage**, not a probabilistic confidence score. The default serial device is `/dev/ttyACM0`.

This preserves the existing demonstrated MVP even if the new capacity feature has not yet been physically commissioned.

## 5. Bin capacity tracking

| Bin | Firmware drop waypoint | Maximum count |
|---|---|---:|
| RED | DR | 3 |
| GREEN | DG | 3 |
| BLUE | DB | 3 |

Each bin uses one fixed drop-off coordinate for all three objects. The firmware owns the calibrated approach and drop coordinates. The Pi configuration records only the bin capacity and destination name. A fourth object is rejected before any serial command.

When upgrading, install the supplied schema-v2 `config/slots.json` and retain `runtime/occupancy.json`. Existing schema-v1 counts and pending operations load directly after validation. If an earlier session changed the Arduino’s in-memory poses, restart the firmware while the arm is safely idle to restore the sketch’s calibrated defaults before using this controller.

For the **capacity-aware** Raspberry Pi workflow:

```bash
# In the project root on Raspberry Pi
python3 Original_Source_Code/RaspberryPi/sort_with_slots.py --verify-config
python3 Original_Source_Code/RaspberryPi/sort_with_slots.py --status
python3 Original_Source_Code/RaspberryPi/sort_with_slots.py
```

Optional serial device override:

```bash
python3 Original_Source_Code/RaspberryPi/sort_with_slots.py --port /dev/ttyACM1
```

Controlled trials without camera recognition may select a color explicitly (they are **not** evidence of automatic color-classification accuracy):

```bash
python3 Original_Source_Code/RaspberryPi/sort_with_slots.py --color GREEN
```

The script sends `CHECK COLOR`, requests approval, saves the pending operation, then sends `ARM` and `RUN COLOR`. After observing the route start and the Arduino’s successful completion message, it increments the selected counter once. This count estimates occupancy. Physical pickup and deposit require observation.

The Pi atomically writes `runtime/occupancy.json`. An interruption after the pending record is saved leaves the count unchanged until reconciliation. Once motion has stopped, inspect the gripper and bin before choosing a recovery command.

```bash
python3 Original_Source_Code/RaspberryPi/sort_with_slots.py --status
# Use ONE of these only after visually checking the intended bin:
python3 Original_Source_Code/RaspberryPi/sort_with_slots.py --resolve-pending placed --confirm
python3 Original_Source_Code/RaspberryPi/sort_with_slots.py --resolve-pending not-placed --confirm
```

After physically emptying a bin, reset its count:

```bash
python3 Original_Source_Code/RaspberryPi/sort_with_slots.py --reset RED --confirm
# or, only after physically emptying all three bins:
python3 Original_Source_Code/RaspberryPi/sort_with_slots.py --reset ALL --confirm
```

Use the same occupancy file for every invocation controlling this arm. An operating-system lock excludes concurrent sorts, resets and recovery commands. Pending operations must be reconciled before resetting any bin.

See [capacity logic and FSM details](docs/SLOT_TRACKING_AND_FSM.md).

## 6. Mathematical model, finite-state machines and verification

For model joint angles `(b, s, e)` in radians and nominal link lengths `L1 = L2 = 12 cm`, relative to the shoulder pivot:

$$
 h=L_1\cos s + L_2\cos(s+e-\pi),\quad
 x=h\sin b,\quad y=h\cos b,\quad
 z=L_1\sin s+L_2\sin(s+e-\pi).
$$

The controller interpolates joint angles by 1 degree per 100 ms, gripper by 1 degree per 30 ms, with 1-second automatic waypoint delays. The original sort route is:

`AP → PICK (close gripper) → AP → HOME → ABOVE_BIN → DROP (open gripper) → ABOVE_BIN → HOME`.

The original Raspberry Pi and Arduino state diagrams are in `docs/diagrams/`. The enhanced FSM covers capacity validation, approval, execution, completion, counter updates and operator recovery. MATLAB exports its transition table and diagram with `seven_slot_fsm`.

**Verified baseline computational results** (not measured physical runtime):

| Color | Samples | Nominal duration | MATLAB/PyBullet interpretation |
|---|---:|---:|---|
| Red | 430 | 45.7 s | Fixed-destination command sequence |
| Green | 404 | 43.1 s | Fixed-destination command sequence |
| Blue | 468 | 49.5 s | Fixed-destination command sequence |

All nine original FK/IK target roundtrips passed numerical consistency tests. The modeled pickup descent has roughly **2.73 cm lateral deviation** under joint-space interpolation, so clearances cannot be inferred from endpoints. Reported PyBullet tool-frame agreement is numerical consistency between models, not evidence of real accuracy or grasp success. The enhanced workflow has hardware-free tests. Repeated physical sorting trials are still needed to measure placement reliability as each bin fills.

## 7. Evidence, limitations and academic submission

The provided project includes unchanged source code, test scripts, command CSVs, finite-state-machine diagrams, and previously captured Simulink scopes. The following are **not yet evidenced by this source package**: physical capacity trials, independent occupancy verification, repeated sorting success rates, photographs of the final physical setup, and the group's demonstration video.

For a complete technical report, include original hardware/wiring photographs, representative camera ROI/classification images, the MATLAB numerical tables/plots, FSM diagrams, Simulink scopes, PyBullet screenshot, and physically measured trial records with success-rate calculations. Do not label hypothetical Simulink lag or scripted PyBullet object attachment as measured hardware behavior.

The presentation notice requires **10 minutes of presentation plus 2–3 minutes of Q&A**. The group leader is to email **the presentation file, project report and capsule demonstration video**, identifying the group number and project title, to `sadek.belamfedel@um6p.ma` with `Oussama.ziadi@um6p.ma` in CC. The stated deadline is **22 November 2026 at 00:00** (start of Sunday; submit Saturday evening to avoid ambiguity). See [evidence and submission checklist](docs/SUBMISSION_CHECKLIST.md).

**Limitations of this MVP:** one-frame/fixed-ROI detection, fixed pickup location, open-loop servos, no collision sensing, no continuous throughput operation, estimated bin occupancy, and only a generic scripted 3D representation. Expansion into industrial operation would require measured calibration, continuous sensing, verified trajectory clearance, reliable power and safety interlocks, mechanical robustness, and throughput testing.

## 8. Attribution

- T. Selçuk Yüksel, [SEVEN 3-DOF Arduino Robotic Arm](https://github.com/elevenMiles/Robotic_Arm_Seven), MIT licensed upstream design.
- MATLAB and Simulink (MathWorks); PyBullet; OpenCV; Raspberry Pi Picamera2; Arduino Servo library.
- The original Arduino sketch and original Raspberry Pi script are retained under `Original_Source_Code`; new work is kept separately and documented as an extension.
