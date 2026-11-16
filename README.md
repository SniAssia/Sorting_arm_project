
# SEVEN — Vision-Guided Color-Sorting Robotic Arm

## Project Context

This project is a working minimum viable product (MVP) for an industrial color sorting system.

**Embedded Systems | UM6P College of Computing**
 
**Group Members:** Assia Snissi Khadija Lekbiri Manal Zaidi Rhita Rhallami

**Project:** Arduino + Raspberry Pi color sorting, MATLAB/Simulink modeling, and PyBullet 3D simulation.

## 1. Project Overview

This project implements an automated color-sorting system using a 3D-printed SEVEN robotic arm, an Arduino Uno, a Raspberry Pi, and a camera.

The Raspberry Pi identifies an object's dominant color using computer vision. The Arduino then controls the robotic arm to pick up the object and place it in the corresponding destination.

Three colors are supported:

- **RED** → Zone A
- **GREEN** → Zone B
- **BLUE** → Zone C

Objects that do not meet the detection threshold are classified as UNKNOWN, and no movement command is sent.

### System Architecture

```text
          Raspberry Pi Camera
                  |
                  v
          Image Acquisition
             (640 x 480)
                  |
                  v
        HSV Color Classification
           (OpenCV + NumPy)
                  |
                  v
         Color Decision Logic
          RED/GREEN/BLUE
                  |
                  v
       Serial Communication
          (USB, 115200)
                  |
                  v
             Arduino Uno
                  |
                  v
          Inverse Kinematics
                  |
                  v
       Finite-State Controller
                  |
                  v
           Four SG90 Servos
                  |
                  v
      Pick-and-Place Operation
```

The system operates with a fixed pickup position and a single-frame color classification. The operator confirms execution before the Arduino begins the sorting sequence.

The mechanical design is based on the open-source [SEVEN robotic arm](https://github.com/elevenMiles/Robotic_Arm_Seven) by T. Selçuk Yüksel, licensed under the MIT License.

Our implementation uses a customized Arduino control program, Raspberry Pi computer vision, mathematical modeling, and simulation.

---

## 2. Implemented Features

### Hardware and Embedded Control

- 3D-printed SEVEN robotic arm
- Three positioning degrees of freedom
- Servo-operated gripper
- Arduino-based joint control
- USB serial communication
- Color-dependent pick-and-place sequences
- Configurable target positions
- Joint-angle interpolation
- Inverse and forward kinematics
- Route reachability checking

### Computer Vision

- Raspberry Pi camera acquisition using Picamera2
- HSV-based color segmentation using OpenCV
- Recognition of red, green, and blue
- Detection coverage threshold
- Fixed region of interest
- Serial communication with the Arduino
- Operator confirmation before execution

### Modeling and Simulation

- MATLAB forward and inverse kinematics
- Mathematical trajectory generation
- Joint-angle and end-effector analysis
- MATLAB finite-state-machine models
- Simulink servo-response simulation
- PyBullet 3D pick-and-place simulation
- Numerical validation tests

The MATLAB and PyBullet implementations simulate the commanded behavior. They do not use measured joint-position feedback.

---

## 3. Project Structure

```text
SEVEN_Color_Sorting_Submission/
|
|-- README.md
|-- .gitignore
|
|-- Original_Source_Code/
|   |-- Arduino/
|   |   |-- SEVEN_color_sorter_original/
|   |   |   |-- SEVEN_color_sorter_original.ino
|   |   |-- Arduino_original_paste.txt
|   |
|   |-- RaspberryPi/
|   |   |-- color_sorter_original.py
|   |
|   |-- SOURCE_MANIFEST.json
|   |-- README.md
|
|-- MATLAB/
|   |-- run_SEVEN_project.m
|   |-- seven_self_test.m
|   |-- seven_fsm.m
|   |-- seven_fsm_self_test.m
|   |-- seven_build_simulink.m
|   |-- src/
|   |-- outputs/
|       |-- RED/
|       |-- GREEN/
|       |-- BLUE/
|
|-- PyBullet/
|   |-- simulate.py
|   |-- test_urdf.py
|   |-- make_urdf.py
|   |-- requirements.txt
|   |-- robot/
|   |   |-- seven_kinematic.urdf
|   |-- trajectories/
|
|-- docs/
|   |-- MODEL_AND_ARCHITECTURE.md
|   |-- FINITE_STATE_MACHINES.md
|   |-- VALIDATION.md
|   |-- SUBMISSION_CHECKLIST.md
|   |-- diagrams/
|   |-- evidence/
|   |-- physical_test_template.csv
|
|-- tools/
    |-- preflight.py
```

The original hardware-control source code is stored separately from the simulation implementations.

---

## 4. Software Requirements

| Component | Requirements |
|---|---|
| MATLAB modeling | MATLAB |
| Servo simulation | MATLAB + Simulink |
| Finite-state machines | MATLAB |
| PyBullet simulation | Python 3, PyBullet, Pillow |
| Arduino control | Arduino IDE, Servo library |
| Raspberry Pi vision | Raspberry Pi OS, Picamera2, OpenCV, NumPy, pyserial |

The project does not require:

- Simscape Multibody
- Robotics System Toolbox
- Control System Toolbox
- Stateflow
- STL files

The 3D simulation uses a generic articulated robotic arm.

---

## 5. Quick Start: Simulation Without Hardware

The numerical models and PyBullet simulation can be reproduced without access to the physical robot.

### Step 1: Download the Project

Clone the repository:

```bash
git clone <YOUR_REPOSITORY_URL>
cd SEVEN_Color_Sorting_Submission
```

Alternatively, download the repository as a ZIP and extract it.

### Step 2: Verify the Existing Trajectories

Open PowerShell in the project directory.

```powershell
py PyBullet/simulate.py --verify-only --all
```

Expected results:

```text
RED: 430 samples, duration 45.7s
GREEN: 404 samples, duration 43.1s
BLUE: 468 samples, duration 49.5s
```

The test also verifies numerical consistency between the exported joint commands and the forward-kinematics equations.

Verify the robot description:

```powershell
py PyBullet/test_urdf.py
```

These commands do not require physical hardware.

### Step 3: Run MATLAB Modeling

Open MATLAB.

Set the Current Folder to:

```text
SEVEN_Color_Sorting_Submission/MATLAB
```

Run:

```matlab
run_SEVEN_project
```

This executes the mathematical modeling workflow:

1. Load robot parameters and target coordinates.
2. Calculate forward and inverse kinematics.
3. Validate the configured target positions.
4. Generate sorting trajectories for all three colors.
5. Calculate trajectory statistics.
6. Export CSV files and figures.

The outputs are saved in:

```text
MATLAB/outputs/
```

Run the mathematical validation tests:

```matlab
seven_self_test
```

Expected result:

```text
RED PASS
GREEN PASS
BLUE PASS

PASS: all FK/IK, route, timing,
and gripper checks.
```

### Step 4: Run the Finite-State-Machine Models

In MATLAB:

```matlab
seven_fsm
```

The script models:

- Raspberry Pi image classification and decision logic
- Arduino route execution
- Joint movement states
- Gripper actions
- Waypoint transitions
- Completion and error handling

Generated outputs include state diagrams, transition tables, and event traces.

Validate the FSM models:

```matlab
seven_fsm_self_test
```

The generated results are written to:

```text
MATLAB/outputs/fsm/
```

Additional documentation is provided in:

[Finite-State-Machine Documentation](docs/FINITE_STATE_MACHINES.md)

### Step 5: Generate the Simulink Model

In MATLAB:

```matlab
seven_build_simulink
```

This generates:

```text
MATLAB/outputs/SEVEN_Servo_Tracking.slx
```

Open the model and run the simulation.

Inspect the three Scope blocks:

- Base_Scope
- Shoulder_Scope
- Elbow_Scope

Each compares the commanded joint angle with a hypothetical first-order servo response.

The simplified servo model is:

\[
G(s)=\frac{1}{\tau s+1}
\]

where the assumed response time constant is:

\[
\tau = 0.35 \text{ s}
\]

This parameter is illustrative and has not been experimentally identified from the physical motors.

### Step 6: Install PyBullet

Open a terminal in the repository root.

Install the dependencies:

```powershell
py -m pip install -r PyBullet/requirements.txt
```

### Step 7: Run the 3D Simulation

Run the RED sorting sequence:

```powershell
py PyBullet/simulate.py --color RED
```

Run GREEN:

```powershell
py PyBullet/simulate.py --color GREEN
```

Run BLUE:

```powershell
py PyBullet/simulate.py --color BLUE
```

To execute all three simulations without opening the graphical viewer:

```powershell
py PyBullet/simulate.py --all --headless
```

To speed up the simulation:

```powershell
py PyBullet/simulate.py --color RED --speed 3
```

To capture a simulation screenshot:

```powershell
py PyBullet/simulate.py --color RED --screenshot
```

Screenshots are saved under:

```text
PyBullet/screenshots/
```

### How the PyBullet Simulation Works

PyBullet imports the generic articulated robot model:

```text
PyBullet/robot/seven_kinematic.urdf
```

The simulator reads the generated joint-command trajectories.

For each sample, it updates:

- Base rotation
- Shoulder rotation
- Elbow rotation
- Gripper opening/closing

The robot follows the same eight-waypoint sorting sequence used by the Arduino model.

Object pickup and release are scripted events. The simulation does not validate gripping force or contact mechanics.

---

## 6. Running the Physical Robot

This section requires the assembled SEVEN robotic arm, Arduino Uno, Raspberry Pi, camera, and appropriate servo power supply.

### 6.1 Hardware Connections

| Component | Arduino Pin |
|---|---|
| Base servo | D6 |
| Shoulder servo | D9 |
| Elbow servo | D10 |
| Gripper servo | D11 |

The four servos require an appropriate external power supply.

The Arduino and servo supply must share a common ground.

Do not power all four motors directly from the Arduino 5V pin.

### 6.2 Upload the Arduino Program

Open Arduino IDE.

Navigate to:

```text
Original_Source_Code/Arduino/
SEVEN_color_sorter_original/
SEVEN_color_sorter_original.ino
```

Select the correct Arduino board and serial port.

Compile and upload the sketch.

The serial communication speed is:

```text
115200 baud
```

The Arduino supports commands including:

| Command | Function |
|---|---|
| ARM | Initialize servo control |
| HOME | Move to home position |
| FK | Display predicted Cartesian position |
| M b s e | Move to specified joint angles |
| XYZ x y z | Move to a Cartesian target |
| CHECK RED | Validate red route |
| CHECK GREEN | Validate green route |
| CHECK BLUE | Validate blue route |
| RUN RED | Execute red sorting route |
| RUN GREEN | Execute green sorting route |
| RUN BLUE | Execute blue sorting route |
| MANUAL RED | Execute route with manual advancement |
| NEXT | Advance manual route |
| STATUS | Display controller state |
| STOP | Cancel motion updates |
| OPEN | Open gripper |
| CLOSE | Close gripper |

**Safety:** The STOP command stops command updates but does not remove servo power. It must not be treated as an emergency power cutoff.

Before using automatic movements, verify the servo calibration, mechanical clearances, and configured target coordinates.

### 6.3 Configure the Raspberry Pi

Connect the Raspberry Pi camera and the Arduino through USB.

The original Python code is located at:

```text
Original_Source_Code/RaspberryPi/
color_sorter_original.py
```

Install the required Python libraries in a compatible Raspberry Pi OS environment.

The code uses:

```python
from picamera2 import Picamera2
import cv2
import numpy as np
import serial
```

The default Arduino connection is:

```python
PORT = "/dev/ttyACM0"
BAUD = 115200
```

Adjust the port in a deployment copy if the Arduino is assigned a different device name.

Run:

```bash
python3 color_sorter_original.py
```

### 6.4 Camera Configuration

The camera captures an RGB888 frame at:

```text
640 x 480 pixels
```

The region of interest is defined as:

```python
x1, y1 = 220, 140
x2, y2 = 420, 340
```

The ROI dimensions are:

```text
200 x 200 pixels
```

The program converts the region into HSV color space and calculates the percentage of pixels belonging to each supported color.

The minimum coverage threshold is:

```python
MIN_COVERAGE = 20.0
```

A color is accepted when its matching pixels cover at least 20% of the region.

If the threshold is not reached, the result is UNKNOWN and no sorting command is sent.

### 6.5 Sorting Execution

The complete workflow is:

1. Position an object at the predefined pickup location.
2. Execute the Raspberry Pi program.
3. Capture an image and classify the object.
4. Send `CHECK COLOR` to Arduino.
5. Verify that the modeled route is reachable.
6. Ask the operator to confirm execution.
7. Send `ARM`.
8. Send `RUN COLOR`.
9. Execute the eight-waypoint pick-and-place sequence.
10. Monitor Arduino messages until completion or error.

The camera script saves:

```text
classification_result.jpg
roi.jpg
```

These images can be used for technical validation after reviewing them for privacy and relevance.

---

## 7. Mathematical Modeling

The robotic arm has three positioning degrees of freedom:

- Base rotation
- Shoulder rotation
- Elbow rotation

A fourth servo operates the gripper.

### 7.1 Robot Geometry

The model uses two nominal arm segments:

\[
L_1=L_2=12\text{ cm}
\]

The coordinate-system origin is the shoulder pivot.

The joint variables are:

\[
q=\begin{bmatrix}b&s&e\end{bmatrix}^{T}
\]

where:

- \(b\): base angle
- \(s\): shoulder angle
- \(e\): elbow angle

### 7.2 Forward Kinematics

The horizontal radial distance is:

\[
h=L_1\cos(s)+L_2\cos(s+e-\pi)
\]

The end-effector coordinates are:

\[
x=h\sin(b)
\]

\[
y=h\cos(b)
\]

\[
z=L_1\sin(s)+L_2\sin(s+e-\pi)
\]

These equations are implemented in Arduino and MATLAB.

### 7.3 Inverse Kinematics

Given a target position:

\[
P=(x,y,z)
\]

the base angle is:

\[
b=\operatorname{atan2}(x,y)
\]

The horizontal distance is:

\[
r=\sqrt{x^2+y^2}
\]

The shoulder-to-target distance is:

\[
d=\sqrt{r^2+z^2}
\]

The elbow angle is computed using the cosine law:

\[
e=\cos^{-1}
\left(
\frac{L_1^2+L_2^2-d^2}{2L_1L_2}
\right)
\]

The shoulder angle is:

\[
s=
\operatorname{atan2}(z,r)+
\cos^{-1}
\left(
\frac{L_1^2+d^2-L_2^2}{2L_1d}
\right)
\]

Joint-limit and reachability checks are applied before commanding the arm.

### 7.4 Sorting Sequence

Each color uses eight waypoints:

```text
1. ABOVE_PICKUP
2. PICK
3. ABOVE_PICKUP
4. HOME
5. ABOVE_DESTINATION
6. DROP
7. ABOVE_DESTINATION
8. HOME
```

The gripper closes at PICK and opens at DROP.

The Arduino interpolates joint angles rather than commanding a perfectly straight Cartesian trajectory.

---

## 8. Finite-State Machines

The system behavior is documented using two state machines.

### 8.1 Raspberry Pi Decision FSM

The Raspberry Pi FSM models:

```text
INITIALIZE
    |
    v
CAPTURE
    |
    v
CLASSIFY
    |
    +---- UNKNOWN ----> NO_COMMAND
    |
    v
CHECK_ROUTE
    |
    v
WAIT_CONFIRMATION
    |
    +---- NO ----> CANCEL
    |
    v
ARM
    |
    v
EXECUTE_ROUTE
    |
    v
MONITOR
    |
    +---- ERROR ----> ERROR
    |
    v
COMPLETE
```

This describes the application-level classification and execution workflow.

### 8.2 Arduino Controller FSM

The Arduino FSM models:

```text
DISARMED
    |
    v
ARMED
    |
    v
MOVING
    |
    v
GRIPPER_ACTION
    |
    v
WAIT_NEXT
    |
    v
NEXT_WAYPOINT
    |
    +---- More waypoints ---> MOVING
    |
    v
COMPLETE
```

The implementation also supports manual advancement, automatic waypoint pauses, invalid-command handling, and STOP behavior.

The complete state-machine specifications and transition tables are documented in:

[FINITE_STATE_MACHINES.md](docs/FINITE_STATE_MACHINES.md)

The MATLAB script:

```matlab
seven_fsm
```

generates the model-level transition tables, state diagrams, and event traces.

The FSM documentation describes the implemented behavior but is not a Stateflow controller or an independent replacement for the Arduino firmware.

---

## 9. Validation Results

### 9.1 Kinematic Validation

The nine configured target positions were checked using:

\[
P \rightarrow IK(P) \rightarrow FK(IK(P))
\]

The numerical reconstruction error is:

\[
\varepsilon =
\left\|P-FK(IK(P))\right\|_2
\]

All nine targets passed the mathematical consistency test, with errors at floating-point roundoff level.

### 9.2 Route Simulation Results

| Color | Samples | Simulated Duration |
|---|---:|---:|
| RED | 430 | 45.7 s |
| GREEN | 404 | 43.1 s |
| BLUE | 468 | 49.5 s |

The durations represent modeled command execution, not measured physical cycle times.

### 9.3 PyBullet Verification

The PyBullet simulation reproduced all three command trajectories.

The previously observed maximum discrepancy between the modeled tool position and the PyBullet robot's tool frame was approximately:

\[
9.9\times10^{-7}\text{ cm}
\]

This verifies agreement between the simulated kinematic implementations, not physical robot accuracy.

### 9.4 Trajectory Limitation

During the modeled pickup descent, the arm exhibits approximately:

\[
\Delta_{xy,\max}=2.73\text{ cm}
\]

of horizontal deviation from the intended vertical pickup line.

This occurs because joint-space interpolation does not necessarily generate straight-line Cartesian motion.

It should be considered when evaluating physical collision clearance.

### 9.5 Experimental Validation

Physical sorting trials should evaluate:

- Successful color classification
- Successful object pickup
- Correct destination placement
- Full sorting cycle duration
- Repeated-operation reliability
- Stability of the power supply

A trial-recording template is available at:

```text
docs/physical_test_template.csv
```

Physical success rates are not reported until the experiments are completed.

---



## 10. References and Credits

**Original robotic arm design**

T. Selçuk Yüksel, *SEVEN — 3-DOF Arduino Robotic Arm*.

Repository: https://github.com/elevenMiles/Robotic_Arm_Seven

License: MIT.

**Software**

- MATLAB and Simulink, MathWorks
- PyBullet, Bullet Physics
- OpenCV
- Picamera2
- Arduino Servo Library

The project adapts the original SEVEN hardware platform and integrates a custom color-sorting workflow, computer vision, mathematical modeling, finite-state-machine documentation, and simulation.

