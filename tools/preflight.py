"""Offline, standard-library-only checks for SEVEN source/model submission.

No MATLAB, camera hardware or PyBullet installation needed for this script.
"""
from __future__ import annotations
import ast
import csv
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent
COLORS = ('RED', 'GREEN', 'BLUE')
BASE = ROOT/'Original_Source_Code'
ARDUINO = BASE/'Arduino'/'SEVEN_color_sorter_original'/'SEVEN_color_sorter_original.ino'
PI = BASE/'RaspberryPi'/'color_sorter_original.py'
# Original controller SHA-256 at commit cb909d0:
# Arduino: 1f847a9d935ca0ec6cbde9c3ca36bd44e92db5dd7b8071cb795dffebd8e5d059
# Pi:      a7305e1119182928d337ba119277d44c894d034d3c92de11c20ec8ed7966eef8
# SHA-256 of the submitted controllers, including comments and line endings.
ARDUINO_SHA256 = 'a31cd5fd2e738ead380fc5e22cfce9f1da74c81e50e389dfbad5c770da74d032'
PI_SHA256 = 'e9a1e7bee25ea544548be9dd8239439b3d168247a7213e848b5177ca93941e22'
FIELDS = ('Time_s','Base_deg','Shoulder_deg','Elbow_deg','Gripper_deg','X_cm','Y_cm','Z_cm')


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fk(q):
    b,s,e=[math.radians(t) for t in q]
    h=12*math.cos(s)+12*math.cos(s+e-math.pi)
    return (h*math.sin(b),h*math.cos(b),12*math.sin(s)+12*math.sin(s+e-math.pi))


def read_commands(path: Path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        missing = set(FIELDS).difference(reader.fieldnames or [])
        assert not missing, f'{path.name} missing columns: {sorted(missing)}'
        return list(reader)


def check_matlab_export(bundled_rows, path: Path):
    generated_rows = read_commands(path)
    assert len(generated_rows) == len(bundled_rows), f'Row count differs in {path}'
    for line, (bundled, generated) in enumerate(zip(bundled_rows, generated_rows), start=2):
        matches = all(
            math.isclose(float(bundled[field]), float(generated[field]), rel_tol=0, abs_tol=1e-8)
            for field in FIELDS
        )
        assert matches and bundled.get('Phase') == generated.get('Phase'), (
            f'MATLAB and bundled commands differ in {path}, line {line}'
        )


def main():
    for file in [ARDUINO, PI]:
        assert file.is_file(), f'Missing source: {file}'
    assert sha256(ARDUINO)==ARDUINO_SHA256, 'Arduino SHA-256 changed'
    assert sha256(PI)==PI_SHA256, 'Raspberry Pi source SHA-256 changed'
    ast.parse(PI.read_text(encoding='utf-8'),filename=str(PI))
    for script in list((ROOT/'PyBullet').glob('*.py')) + [BASE/'RaspberryPi'/'sort_with_slots.py']:
        ast.parse(script.read_text(encoding='utf-8'),filename=str(script))
    for needle in ['CHECK {detected_color}', 'RUN {detected_color}', 'MIN_COVERAGE = 20.0']:
        assert needle in PI.read_text(encoding='utf-8'), f'Pi source missing {needle}'
    print('[PASS] Arduino and Raspberry Pi source checksums match')

    for rel in ['MATLAB/seven_fsm.m','MATLAB/seven_fsm_self_test.m',
                'MATLAB/src/seven_plot_results.m','docs/FINITE_STATE_MACHINES.md',
                'MATLAB/seven_build_simulink.m','PyBullet/robot/seven_kinematic.urdf']:
        assert (ROOT/rel).is_file(), f'Missing required file: {rel}'
    print('[PASS] Required model, script and documentation files')

    urdf=ET.parse(ROOT/'PyBullet/robot/seven_kinematic.urdf').getroot()
    joints={j.attrib['name'] for j in urdf.findall('joint')}
    expected={'base_yaw','shoulder_pitch','elbow_pitch','left_grip','right_grip','tool_fixed'}
    assert expected<=joints, f'Missing generic arm URDF joints: {expected-joints}'
    print('[PASS] Generic URDF joint structure')

    for color in COLORS:
        bundled=ROOT/f'PyBullet/trajectories/{color}_commands.csv'
        assert bundled.is_file(), f'Missing bundled {color} route'
        csvrows=read_commands(bundled)
        assert len(csvrows)>=200, f'Short route {color}'
        times=[]; close_t=None; open_t=None; max_error=0.0
        for r in csvrows:
            for key in FIELDS:
                assert math.isfinite(float(r[key])), f'{color} non-finite value {key}'
            t=float(r['Time_s']); times.append(t)
            q=[float(r[k]) for k in ['Base_deg','Shoulder_deg','Elbow_deg']]
            xyz=fk(q)
            xyz_csv=[float(r[k]) for k in ['X_cm','Y_cm','Z_cm']]
            max_error=max(max_error,math.dist(xyz,xyz_csv))
            grip=float(r['Gripper_deg'])
            if close_t is None and grip>=115-1e-6: close_t=t
            elif close_t is not None and open_t is None and grip<=90+1e-6: open_t=t
        assert all(v>u for u,v in zip(times,times[1:])), f'Nonincreasing timestamps in {color}'
        assert close_t is not None and open_t is not None, f'Grip/release missing in {color}'
        assert max_error<0.02, f'{color} FK numerical deviation {max_error} cm'  # CSV precision
        print(f'[PASS] {color}: {len(csvrows)} samples, {times[-1]:.1f}s; grip {close_t:.1f}s, release {open_t:.1f}s; FK max {max_error:.8f}cm')
        generated=ROOT/f'MATLAB/outputs/{color}/commands.csv'
        if generated.exists():
            check_matlab_export(csvrows, generated)
            print(f'[PASS] {color}: local MATLAB export matches the bundled commands')

    config=json.loads((ROOT/'config/slots.json').read_text(encoding='utf-8'))
    assert config['schema_version']==2
    assert config['bins']=={color: {'capacity':3, 'destination':dest}
                            for color,dest in zip(COLORS, ('DR','DG','DB'))}
    for rel in ['MATLAB/seven_slot_fsm.m','MATLAB/seven_slot_fsm_self_test.m',
                'Original_Source_Code/RaspberryPi/sort_with_slots.py', 'tests/test_slot_sorter.py']:
        assert (ROOT/rel).is_file(), f'Missing bin capacity component: {rel}'
    print('[PASS] Three bins of capacity three reference the original firmware destinations')

    docs=['README.md','docs/SLOT_TRACKING_AND_FSM.md','docs/MODEL_AND_ARCHITECTURE.md',
          'docs/FINITE_STATE_MACHINES.md']
    for rel in docs:
        assert (ROOT/rel).is_file() and (ROOT/rel).stat().st_size>300, f'Missing documentation {rel}'
    print('PASS: offline integrity, simulation consistency and documentation checks complete.')


if __name__=='__main__':
    main()
