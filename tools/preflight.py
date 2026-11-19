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
ARDUINO_COPY = BASE/'Arduino'/'Arduino_original_paste.txt'
PI = BASE/'RaspberryPi'/'color_sorter_original.py'
MANIFEST = BASE/'SOURCE_MANIFEST.json'
FIELDS = ('Time_s','Base_deg','Shoulder_deg','Elbow_deg','Gripper_deg','X_cm','Y_cm','Z_cm')


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fk(q):
    b,s,e=[math.radians(t) for t in q]
    h=12*math.cos(s)+12*math.cos(s+e-math.pi)
    return (h*math.sin(b),h*math.cos(b),12*math.sin(s)+12*math.sin(s+e-math.pi))


def main():
    for file in [ARDUINO, ARDUINO_COPY, PI, MANIFEST]:
        assert file.is_file(), f'Missing source: {file}'
    data=json.loads(MANIFEST.read_text(encoding='utf-8'))
    assert ARDUINO.read_bytes()==ARDUINO_COPY.read_bytes(), 'Arduino byte-for-byte copy differs'
    assert sha256(ARDUINO)==data['arduino_original_sha256'], 'Arduino SHA-256 changed'
    assert sha256(PI)==data['raspberry_pi_transcription_sha256'], 'Pi transcription SHA-256 changed'
    ast.parse(PI.read_text(encoding='utf-8'),filename=str(PI))
    for script in list((ROOT/'PyBullet').glob('*.py')) + [BASE/'RaspberryPi'/'sort_with_slots.py']:
        ast.parse(script.read_text(encoding='utf-8'),filename=str(script))
    for needle in ['CHECK {detected_color}', 'RUN {detected_color}', 'MIN_COVERAGE = 20.0']:
        assert needle in PI.read_text(encoding='utf-8'), f'Pi source missing {needle}'
    print('[PASS] Original Arduino binary identical to preserved paste, both original-source SHA-256 signatures match')

    excluded=['MATLAB/seven_render_3d.m','MATLAB/seven_drive_simscape.m',
              'MATLAB/seven_import_simscape.m','MATLAB/src/seven_animate.m',
              'MATLAB/seven_stl_gallery.m','PyBullet/seven_stl_tools.py','STL_files']
    for rel in excluded:
        assert not (ROOT/rel).exists(), f'Obsolete 3D/Simscape/STL component still included: {rel}'
    for rel in ['MATLAB/seven_fsm.m','MATLAB/seven_fsm_self_test.m',
                'MATLAB/src/seven_plot_results.m','docs/FINITE_STATE_MACHINES.md',
                'MATLAB/seven_build_simulink.m','PyBullet/robot/seven_kinematic.urdf']:
        assert (ROOT/rel).is_file(), f'Missing required file: {rel}'
    print('[PASS] Package is STL-free; PyBullet is the only 3D robot renderer; MATLAB FSM files included')

    urdf=ET.parse(ROOT/'PyBullet/robot/seven_kinematic.urdf').getroot()
    joints={j.attrib['name'] for j in urdf.findall('joint')}
    expected={'base_yaw','shoulder_pitch','elbow_pitch','left_grip','right_grip','tool_fixed'}
    assert expected<=joints, f'Missing generic arm URDF joints: {expected-joints}'
    print('[PASS] Generic URDF joint structure')

    for color in COLORS:
        m=ROOT/f'MATLAB/outputs/{color}/commands.csv'
        p=ROOT/f'PyBullet/trajectories/{color}_commands.csv'
        assert m.is_file() and p.is_file(), f'Missing route {color}'
        assert m.read_bytes()==p.read_bytes(), f'MATLAB/PyBullet CSV disagreement {color}'
        with m.open(newline='',encoding='utf-8-sig') as f:
            csvrows=list(csv.DictReader(f))
        assert len(csvrows)>=200, f'Short route {color}'
        times=[]; close_t=None; open_t=None; max_error=0.0
        for r in csvrows:
            for key in FIELDS:
                assert key in r, f'CSV {color} missing {key}'
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

    config=json.loads((ROOT/'config/slots.json').read_text(encoding='utf-8'))
    assert config['schema_version']==2
    assert config['bins']=={color: {'capacity':3, 'destination':dest}
                            for color,dest in zip(COLORS, ('DR','DG','DB'))}
    for rel in ['MATLAB/seven_slot_fsm.m','MATLAB/seven_slot_fsm_self_test.m',
                'Original_Source_Code/RaspberryPi/sort_with_slots.py', 'tests/test_slot_sorter.py']:
        assert (ROOT/rel).is_file(), f'Missing counter extension: {rel}'
    assert not (ROOT/'START_HERE.md').exists(), 'Root README must be sole setup guide'
    print('[PASS] Three bins of capacity three reference the original firmware destinations')

    docs=['README.md','docs/SLOT_TRACKING_AND_FSM.md','docs/MODEL_AND_ARCHITECTURE.md',
          'docs/FINITE_STATE_MACHINES.md','docs/VALIDATION.md','docs/SUBMISSION_CHECKLIST.md']
    for rel in docs:
        assert (ROOT/rel).is_file() and (ROOT/rel).stat().st_size>300, f'Missing documentation {rel}'
    print('PASS: offline integrity, simulation consistency and documentation checks complete.')
    print('NOTE: MATLAB/Simulink GUI and Raspberry Pi hardware must be validated on the team computers.')


if __name__=='__main__':
    main()
