
## 11. Evidence and Documentation

The final technical report should include the following evidence.

| Evidence | Source |
|---|---|
| Assembled robotic arm | Team photographs |
| Hardware connections | Wiring photographs or circuit diagram |
| Color detection examples | Raspberry Pi camera outputs |
| Kinematic validation | MATLAB output tables |
| Joint trajectories | MATLAB figures |
| State-machine diagrams | `docs/diagrams/` |
| Simulink results | Joint-response scope screenshots |
| PyBullet simulation | 3D simulation screenshot |
| Physical sorting performance | Completed experimental trial records |
| End-to-end demonstration | Team-recorded capsule video |

Simulation figures must be clearly distinguished from measured physical results.

Selected and reviewed evidence can be placed in:

```text
docs/evidence/
```

Unreviewed camera images, private recordings, and local machine files should not be committed to GitHub.


# SEVEN — installation, execution, and verification

Extract the ZIP to a short writable folder on Windows, e.g. `C:\Projects\SEVEN_Color_Sorting_Submission`. The project runs offline except for installing dependencies. **No robot or Raspberry Pi is required for the modeling and PyBullet demonstrations.**

## A. Verify the package first (Windows PowerShell)

Open PowerShell in the extracted project root. You can use the one-command verification shortcut:

```powershell
powershell -ExecutionPolicy Bypass -File .\RUN_CHECKS.ps1
```

Or run the individual checks:

```powershell
py tools\preflight.py
cd PyBullet
py simulate.py --verify-only --all
py test_urdf.py
cd ..
```

Expected: source fingerprints, URDF structure, all three CSV command sets, and Python transform-chain checks pass. A `--verify-only` run does **not** require the `pybullet` package.

## B. MATLAB mathematical model and FSM

Open MATLAB with **Current Folder** set to `...\SEVEN_Color_Sorting_Submission\MATLAB`, then run commands in this order:

```matlab
run_SEVEN_project
seven_self_test
seven_fsm
seven_fsm_self_test
```

Expected outputs:

- `MATLAB/outputs/pose_validation.csv` and `route_summary.csv`.
- `MATLAB/outputs/RED/`, `GREEN/`, `BLUE/`: `commands.csv`, `waypoints.csv`, `joint_angles.png`, `gripper_commands.png`, `xy_footprint.png`, `z_over_time.png`.
- `MATLAB/outputs/fsm/`: `raspberry_pi_transitions.csv`, `arduino_transitions.csv`, FSM diagrams, `trace_RED.csv`, `trace_GREEN.csv`, `trace_BLUE.csv`, and 2D state timelines.

These are **commanded/model outputs**, not camera imagery or measured motor feedback. They use base MATLAB without Stateflow, Robotics Toolbox, or Control System Toolbox. The FSM diagrams are specifications/command traces, not a live Stateflow controller connected to Arduino.

### Simulink scopes

Simulink is required only for this step, and must be installed and licensed. Continue in MATLAB:

```matlab
seven_build_simulink
```

Opens and saves `MATLAB/outputs/SEVEN_Servo_Tracking.slx`. Click **Run** in the model, then open `Base_Scope`, `Shoulder_Scope`, `Elbow_Scope`. The scope plots compare Arduino **commanded** angles against a **hypothetical first-order servo response** using `tau=0.35 s`. They do not measure actual SG90 servo dynamics. Use your existing camera/physical recordings as separate evidence.

## C. PyBullet: the only 3D robot simulation

Return to PowerShell at the project root:

```powershell
cd PyBullet
py -m pip install -r requirements.txt
py simulate.py --color RED
py simulate.py --color GREEN
py simulate.py --color BLUE
```

Use `--speed 3` for slower playback or `--speed 10` for faster playback. The graphical window closes after each completed playback. To check all routes without a GUI:

```powershell
py simulate.py --all --headless
```

To save a sample PyBullet frame:

```powershell
py simulate.py --color RED --screenshot
```

Images go to `PyBullet/screenshots/` (generated at runtime). The robot uses a **generic 3-DOF URDF** and two visual gripper jaws; it **does not require STL files**. Pickup and release are scripted, not determined from contact/force simulation. In GUI mode, the previously encountered cleanup warning is guarded against a disconnected physics server.

PyBullet reads fresh `MATLAB/outputs/COLOR/commands.csv` **when present**, otherwise the bundled `PyBullet/trajectories/COLOR_commands.csv`. Both sources agree in the delivered package. If you change geometry or routes in MATLAB, rerun `run_SEVEN_project` before the PyBullet replay.

## D. Optional physical hardware demonstration

The original control programs are kept untouched in `Original_Source_Code/`. Use **deployment copies** if you need to change ports or calibration, keeping the included originals unchanged.

1. Upload `Original_Source_Code/Arduino/SEVEN_color_sorter_original/SEVEN_color_sorter_original.ino` to an Arduino Uno using Arduino IDE, Servo library, and a compatible board.
2. Connect base/shoulder/elbow/gripper signals to **D6/D9/D10/D11**, respectively. Use a suitable independent servo power supply and **common GND** with Arduino. Verify arm alignment and no mechanical interference before `ARM`.
3. On a properly configured Raspberry Pi, connect Pi Camera and Arduino USB. Install Picamera2/OpenCV/numpy/pyserial according to the Pi OS environment. Check the actual Arduino device (the source defaults to `/dev/ttyACM0`).
4. Copy and run `Original_Source_Code/RaspberryPi/color_sorter_original.py` on the Pi:

```bash
python3 color_sorter_original.py
```

5. Place one RED/GREEN/BLUE object at the **fixed pickup point** inside the camera ROI; inspect the printed color coverage. The program checks the route and **asks for `yes`** before commanding movement. Ensure clear space around the robot.
6. Document repeated physical trials in `docs/physical_test_template.csv`. Run UNKNOWN/no-color trials separately and record whether commands are correctly withheld.

**Safety:** `STOP` is a software command that holds current servo outputs, **not** a physical emergency power disconnect. Avoid obstructing servos and never assume a successful FK check proves collision clearance.

## E. Complete the submission

See [docs/SUBMISSION_CHECKLIST.md](docs/SUBMISSION_CHECKLIST.md) for the group information, physical validation, report, slides, capsule video and email checklist. The project source and simulated evidence are prepared; empirical results and the final presentation artifacts must be completed by your team.
