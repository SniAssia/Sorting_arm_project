"""SEVEN kinematic color sorting replay in PyBullet (no physical arm needed).

Commands:
  python simulate.py --color RED
  python simulate.py --color GREEN --speed 5
  python simulate.py --all --headless
  python simulate.py --verify-only
  python simulate.py --color BLUE --screenshot

The simulation reads MATLAB's command CSV directly (or bundled exports from
actual MATLAB run). The gripper object is attached/released by a SCRIPT, not
PyBullet contact forces. A 3D parametric proxy is NOT an assembled CAD mesh.
"""
from __future__ import annotations
import argparse
import csv
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent
L1, L2 = 0.12, 0.12
SHOULDER_HEIGHT = 0.06
PICK_CM = (3.93, 14.68, -4.0)
DROP_CM = {
    "RED": (13.14, 6.13, 2.0),
    "GREEN": (13.14, -6.13, 2.0),
    "BLUE": (3.7528, -14.0055, 2.0),
}
COLORS = {"RED": (0.92, 0.16, 0.20, 1.0),
          "GREEN": (0.10, 0.67, 0.36, 1.0),
          "BLUE": (0.10, 0.33, 0.91, 1.0)}
COLORS_FLOOR = {c: (r, g, b, 0.35) for c,(r,g,b,a) in COLORS.items()}
COLOR_CHOICES = ["RED", "GREEN", "BLUE"]


def expected_tip(q_deg):
    """Expected tool position relative to shoulder in *meters*."""
    b, s, e = [math.radians(v) for v in q_deg]
    h = L1*math.cos(s) + L2*math.cos(s+e-math.pi)
    return (h*math.sin(b), h*math.cos(b),
            L1*math.sin(s) + L2*math.sin(s+e-math.pi))


def load_rows(color, file_override=None):
    if file_override:
        source = Path(file_override).expanduser().resolve()
    else:
        # If user has run the MATLAB project in this ZIP, use their fresh results.
        generated = ROOT.parent / "MATLAB" / "outputs" / color / "commands.csv"
        source = generated if generated.exists() else ROOT / "trajectories" / f"{color}_commands.csv"
    if not source.exists():
        raise FileNotFoundError(f"No trajectory: {source}. Run run_SEVEN_project in MATLAB first.")
    rows = []
    with source.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = {"Time_s", "Base_deg", "Shoulder_deg", "Elbow_deg", "Gripper_deg",
                    "X_cm", "Y_cm", "Z_cm"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing CSV columns: {sorted(missing)} in {source}")
        for line in reader:
            r = {k: float(line[k]) for k in required}
            r["Phase"] = line.get("Phase", "")
            if any(not math.isfinite(v) for k,v in r.items() if k!="Phase"):
                raise ValueError("CSV contains nonfinite values")
            rows.append(r)
    if len(rows) < 8:
        raise ValueError("Trajectory must have at least 8 rows")
    return rows, source


def verify_csv_math(color, rows):
    max_error = 0.0
    prev_t = -math.inf
    grab = release = None
    for r in rows:
        if r["Time_s"] <= prev_t:
            raise ValueError(f"{color}: time samples are not strictly increasing")
        prev_t = r["Time_s"]
        q = [r["Base_deg"], r["Shoulder_deg"], r["Elbow_deg"]]
        expected = expected_tip(q)
        given = [r["X_cm"]/100, r["Y_cm"]/100, r["Z_cm"]/100]
        error = math.dist(expected, given)
        max_error = max(max_error, error)
        if grab is None and r["Gripper_deg"] >= 115 - 1e-5:
            grab = r["Time_s"]
        elif grab is not None and release is None and r["Gripper_deg"] <= 90 + 1e-5:
            release = r["Time_s"]
    if max_error > 0.0002:
        raise AssertionError(f"{color} MATLAB vs Python FK difference: {max_error*100:.4f} cm")
    if grab is None or release is None:
        raise AssertionError(f"{color} route did not complete gripper-close and gripper-open events")
    print(f"{color}: {len(rows)} samples, duration {rows[-1]['Time_s']:.1f}s, "
          f"max FK numerical mismatch {max_error*100:.8f}cm, "
          f"grasp at {grab:.1f}s, release at {release:.1f}s")
    return grab, release


def import_bullet():
    try:
        import pybullet as p
    except ModuleNotFoundError as e:
        raise SystemExit("PyBullet is not installed in this Python environment. "
                         "Run: python -m pip install pybullet pillow") from e
    return p


def visual_box(p, xyz, size, color):
    shape = p.createVisualShape(p.GEOM_BOX,halfExtents=[v/2 for v in size],rgbaColor=color)
    return p.createMultiBody(baseMass=0,baseVisualShapeIndex=shape,basePosition=xyz)


def make_bin(p, xy, rgba):
    x,y=xy
    w,d,h=0.057,0.057,0.061
    bottom=0.024
    thickness=0.004
    visual_box(p,(x,y,bottom),(w,d,thickness),rgba)
    for sign in (-1,1):
        visual_box(p,(x+sign*w/2,y,bottom+h/2),(thickness,d,h),rgba)
        visual_box(p,(x,y+sign*d/2,bottom+h/2),(w,thickness,h),rgba)


def setup_scene(p,color,headless=False):
    mode=p.DIRECT if headless else p.GUI
    client=p.connect(mode)
    if client < 0:
        raise RuntimeError("Unable to open PyBullet. Try --headless for a headless verification run")
    p.setGravity(0,0,-9.81)
    p.setTimeStep(1/240)
    p.setRealTimeSimulation(0)
    if not headless:
        try:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
            p.resetDebugVisualizerCamera(cameraDistance=0.65,cameraYaw=45,
                                         cameraPitch=-35,cameraTargetPosition=(0.06,0.0,0.07))
        except Exception:
            pass
    visual_box(p,(0,0,-.008),(0.68,0.64,.016),(.88,.90,.92,1))
    for c in COLOR_CHOICES:
        xy=[v/100 for v in DROP_CM[c][:2]]
        rgba=COLORS[c]
        make_bin(p,xy,(rgba[0],rgba[1],rgba[2],.45))
    urdf=ROOT / "robot" / "seven_kinematic.urdf"
    if not urdf.exists():
        from make_urdf import generate
        generate()
    robot=p.loadURDF(str(urdf),basePosition=(0,0,0),useFixedBase=True)
    joints={p.getJointInfo(robot,j)[1].decode("utf-8"):j
            for j in range(p.getNumJoints(robot))}
    expected={"base_yaw", "shoulder_pitch", "elbow_pitch",
              "left_grip", "right_grip", "tool_fixed"}
    if not expected.issubset(joints):
        raise ValueError(f"URDF missing joints: {sorted(expected.difference(joints))}")
    shape=p.createVisualShape(p.GEOM_BOX,halfExtents=(.012,.012,.012),rgbaColor=COLORS[color])
    obj=p.createMultiBody(baseMass=0,baseVisualShapeIndex=shape,
                          basePosition=(PICK_CM[0]/100,PICK_CM[1]/100,
                                        SHOULDER_HEIGHT+PICK_CM[2]/100))
    # Display path as a dotted line at a few anchors when requested.
    return robot,joints,obj


def reset_arm(p,robot,joints,row):
    b,s,e = [row[x] for x in ["Base_deg", "Shoulder_deg", "Elbow_deg"]]
    p.resetJointState(robot,joints["base_yaw"],-math.radians(b))
    p.resetJointState(robot,joints["shoulder_pitch"],math.radians(s))
    p.resetJointState(robot,joints["elbow_pitch"],math.radians(e)-math.pi)
    fraction=(row["Gripper_deg"]-90.0)/25.0
    fraction=max(0.0,min(1.0,fraction))
    separation=0.043-0.025*fraction
    p.resetJointState(robot,joints["left_grip"],separation/2)
    p.resetJointState(robot,joints["right_grip"],-separation/2)
    state=p.getLinkState(robot,joints["tool_fixed"],computeForwardKinematics=1)
    return state[4] # world link reference position (not COM position)


def camera_snapshot(p,filepath):
    # Tiny renderer works in DIRECT mode and does not depend on screen capture.
    width,height=1024,720
    view=p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.055,0,0.065],
                distance=0.64,yaw=45,pitch=-33,roll=0,upAxisIndex=2)
    proj=p.computeProjectionMatrixFOV(fov=55,aspect=width/height,nearVal=.01,farVal=3.0)
    _,_,raw,_,_=p.getCameraImage(width,height,viewMatrix=view,projectionMatrix=proj,
                                    renderer=p.ER_TINY_RENDERER)
    try:
        from PIL import Image
        rgba=raw.tobytes() if hasattr(raw,"tobytes") else bytes(raw)
        Image.frombytes("RGBA",(width,height),rgba).convert("RGB").save(filepath)
        print("Saved snapshot:",filepath)
    except ImportError:
        print("Pillow not installed; to save PNG run: python -m pip install pillow")


def simulate(color,rows,headless=False,speed=6.0,screenshot=False,show_trail=True):
    p=import_bullet()
    robot=joints=obj=None
    p_client=None
    try:
        robot,joints,obj=setup_scene(p,color,headless=headless)
        p_client=0
        if show_trail:
            trajectory=[(r["X_cm"]/100,r["Y_cm"]/100,
                         SHOULDER_HEIGHT+r["Z_cm"]/100) for r in rows]
            rgb=COLORS[color][:3]
            stride=max(1,len(rows)//130)
            for k in range(0,len(rows)-stride,stride):
                p.addUserDebugLine(trajectory[k],trajectory[k+stride],lineColorRGB=rgb,
                                   lineWidth=1.6,lifeTime=0)
        grab_idx=next(i for i,r in enumerate(rows) if r["Gripper_deg"]>=115-1e-5)
        release_idx=next(i for i,r in enumerate(rows)
                         if i>grab_idx and r["Gripper_deg"]<=90+1e-5)
        snapshot_index=int(len(rows)*.70)
        max_tip_error=0.0
        for i,row in enumerate(rows):
            tip=reset_arm(p,robot,joints,row)
            expected=expected_tip([row[k] for k in ("Base_deg","Shoulder_deg","Elbow_deg")])
            world_expected=(expected[0],expected[1],expected[2]+SHOULDER_HEIGHT)
            tip_error=math.dist(tip,world_expected)
            max_tip_error=max(max_tip_error,tip_error)
            if tip_error>.001:
                raise AssertionError("PyBullet FK disagrees with Arduino model by "
                                     f"{tip_error*100:.3f} cm at sample {i}. "
                                     "Check joint signs and URDF geometry.")
            if i<grab_idx:
                cube=(PICK_CM[0]/100,PICK_CM[1]/100,SHOULDER_HEIGHT+PICK_CM[2]/100)
            elif i<release_idx:
                cube=tip # PRESCRIBED attachment to end-effector
            else:
                drop=DROP_CM[color]
                cube=(drop[0]/100,drop[1]/100,.045) # settled in a modeled bin
            p.resetBasePositionAndOrientation(obj,cube,(0,0,0,1))
            if i == grab_idx: print(color,": scripted grasp triggered")
            if i == release_idx: print(color,": scripted release triggered")
            # Do not step dynamics here: this is deterministic commanded-kinematics
            # playback. A physics step would move joints under gravity/default motors.
            if screenshot and i==snapshot_index:
                out=ROOT / "screenshots"
                out.mkdir(exist_ok=True)
                camera_snapshot(p,str(out/f"SEVEN_PyBullet_{color}.png"))
            if not headless and i>0:
                dt=max(0.0, row["Time_s"]-rows[i-1]["Time_s"])
                time.sleep(min(dt/speed,.04))
        print(f"{color}: PyBullet robot tool FK max mismatch: {max_tip_error*100:.8f} cm")
        if not headless:
            print("Complete. Closing the viewer shortly (Ctrl+C to exit immediately).")
            time.sleep(3)
    finally:
        if p_client is not None:
            try:
                if p.isConnected():
                    p.disconnect()
            except p.error:
                pass  # GUI/physics server may already have closed


def cli(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--color",choices=COLOR_CHOICES,default="RED")
    parser.add_argument("--all",action="store_true",help="Play all three colors one after the other")
    parser.add_argument("--headless",action="store_true",help="Run in DIRECT mode (no GUI)")
    parser.add_argument("--speed",type=float,default=6.0,help="GUI playback speed multiplier (default 6x)")
    parser.add_argument("--verify-only",action="store_true",help="Numerical CSV checks; does NOT require PyBullet")
    parser.add_argument("--csv",type=str,help="Use an explicit MATLAB commands.csv (single color only)")
    parser.add_argument("--screenshot",action="store_true",help="Save simulated screenshot at 70%% of route")
    parser.add_argument("--no-trail",action="store_true",help="Do not draw the command path")
    args=parser.parse_args(argv)
    if args.speed <= 0:
        parser.error("--speed must be positive")
    if args.all and args.csv:
        parser.error("--csv cannot be combined with --all")
    colors=COLOR_CHOICES if args.all else [args.color]
    for c in colors:
        rows,path=load_rows(c,args.csv)
        print("Trajectory source:",path)
        verify_csv_math(c,rows)
        if not args.verify_only:
            simulate(c,rows,headless=args.headless,speed=args.speed,
                     screenshot=args.screenshot,show_trail=not args.no_trail)


if __name__=="__main__":
    cli()
