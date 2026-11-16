# Architecture and mathematical model

## 1. Hardware/software boundaries

```text
Pi Camera -> Picamera2 RGB888 capture (640x480)
          -> fixed ROI [x:220..420, y:140..340]
          -> OpenCV HSV red/green/blue masks and pixel coverage
          -> color if >=20% coverage, else UNKNOWN
          -> USB serial CHECK COLOR (115200 baud), parse route-check text
          -> operator confirmation -> ARM -> RUN COLOR
          -> Arduino route FSM / IK + joint-command interpolation
          -> D6 base, D9 shoulder, D10 elbow, D11 gripper
          -> pickup; lift; HOME; colored bin; release; HOME
```

The camera captures **one** frame per execution. The classification code has separate low/high-hue ranges for RED; the `confidence` variable is HSV mask **coverage**, not a classifier probability. Raspberry Pi script captures the frame, writes `classification_result.jpg` and `roi.jpg`, opens serial only after a color passes threshold, checks the route, requests confirmation, and monitors textual Arduino responses. The serial `ARM` command is sent before `RUN`; the Pi does not require a distinct positive `ARM` acknowledgment before transmitting `RUN`.

## 2. Geometric definitions

The origin is the shoulder-pivot point, not the camera or robot-base origin. Link lengths are nominal `L1=L2=12 cm`. Joint variables are `(b,s,e)` in degrees at the firmware interface, converted to radians in trig calculations.

\[
\begin{aligned}
 h &= L_1\cos(s)+L_2\cos(s+e-\pi),\\
 x &= h\sin(b),\\
 y &= h\cos(b),\\
 z &= L_1\sin(s)+L_2\sin(s+e-\pi).
\end{aligned}
\]

Inverse kinematics, following the Arduino source, uses:

\[
 r=\sqrt{x^2+y^2},\qquad d=\sqrt{r^2+z^2},\qquad b=\operatorname{atan2}(x,y),
\]

\[
 e=\arccos\left(\frac{L_1^2+L_2^2-d^2}{2L_1L_2}\right),\quad
 s=\operatorname{atan2}(z,r)+\arccos\left(\frac{L_1^2+d^2-L_2^2}{2L_1d}\right).
\]

The firmware rejects unreachable points, including additional conservative software conditions `x>=1 cm`, `z>=-6 cm`, `3 cm<=d<=23 cm`, and model joint angles in `[0,180]`. These conditions do **not** establish obstacle avoidance or physical mechanical clearances.

## 3. Waypoint order and command timing

The route in the team's custom firmware is:

| Step | Commanded target | Gripper action after target |
|---:|---|---|
| 1 | `AP` above pickup | none |
| 2 | `PICK` | CLOSE |
| 3 | `AP` after lifting | none |
| 4 | `HOME` | none |
| 5 | `AR/AG/AB` above selected bin | none |
| 6 | `DR/DG/DB` at selected bin | OPEN |
| 7 | `AR/AG/AB` above bin | none |
| 8 | `HOME` | none, sequence ends |

The controller increments each commanded joint by up to 1° every 100 ms. It increments the gripper by 1° every 30 ms and pauses 1000 ms between automatic waypoints. `RUN COLOR` is automatically advanced; `MANUAL COLOR` requires `NEXT` after each waypoint. MATLAB simulates the `RUN` command, starting from the firmware's initial `ARM` pose `[105,73,60]` degrees. It does not model unknown serial/camera delays.

Joint-space motion causes curved paths in Cartesian space even when two endpoints have equal XY coordinates. The numerical pickup descent deviates by approximately **2.73 cm** in XY from the intended vertical line. Clearance must be physically checked across the entire sweep.

## 4. Simulink interpretation

For explanatory purposes only, the servo actuator is represented as an assumed first-order system:

\[
 G(s)=\frac{\theta_{\mathrm{hypothetical}}(s)}{\theta_{\mathrm{command}}(s)}
 =\frac{1}{\tau s+1},\qquad \tau=0.35\,\mathrm{s}\;\text{(assumed)}.
\]

The values of tau, torque, friction and servo gain are **not experimentally identified**. The position curve is a numerical prediction, not encoder measurements. Full rigid-body dynamics and a physically validated gripper are outside this project's computational model.

## 5. PyBullet mapping

The generic robot URDF uses metres/radians, with a modeled shoulder height of `0.06 m` above the PyBullet floor. Joint-state transforms are:

- `base_yaw = -deg2rad(b)`
- `shoulder_pitch = deg2rad(s)`
- `elbow_pitch = deg2rad(e) - pi`

The two virtual prismatic jaws are a **visual approximation** of the physical servo gripper. A colored cube is programmatically attached to the commanded end-effector position and released at the destination. Physics-based contact, forces, collisions and real servo actuation have **not** been validated by these animations.
