"""Pure Python sanity checks for the generated URDF and exported MATLAB commands."""
import math
from pathlib import Path
from xml.etree import ElementTree as ET
from simulate import expected_tip, load_rows, verify_csv_math

ROOT=Path(__file__).resolve().parent
urdf=ET.parse(ROOT/'robot'/'seven_kinematic.urdf').getroot()
joints={j.get('name'):j for j in urdf.findall('joint')}
assert set(['base_yaw','shoulder_pitch','elbow_pitch','wrist_mount',
            'left_grip','right_grip','tool_fixed']).issubset(joints)
assert abs(float(joints['base_yaw'].find('origin').get('xyz').split()[2])-0.06)<1e-12
assert abs(float(joints['elbow_pitch'].find('origin').get('xyz').split()[1])-0.12)<1e-12
assert abs(float(joints['wrist_mount'].find('origin').get('xyz').split()[1])-0.12)<1e-12
assert joints['shoulder_pitch'].find('axis').get('xyz')=='1 0 0'
assert joints['elbow_pitch'].find('axis').get('xyz')=='1 0 0'

# Independent rigid-frame composition checks vs the Arduino formula.
def rx(a,p):
 x,y,z=p
 return (x,math.cos(a)*y-math.sin(a)*z,math.sin(a)*y+math.cos(a)*z)
def rz(a,p):
 x,y,z=p
 return (math.cos(a)*x-math.sin(a)*y, math.sin(a)*x+math.cos(a)*y,z)
for col in ['RED','GREEN','BLUE']:
 rows,_=load_rows(col)
 verify_csv_math(col,rows)
 for r in rows:
  b,s,e=[math.radians(r[c]) for c in ('Base_deg','Shoulder_deg','Elbow_deg')]
  l1=rz(-b,rx(s,(0,.12,0)))
  l2=rz(-b,rx(s+e-math.pi,(0,.12,0)))
  composed=tuple(l1[k]+l2[k] for k in range(3))
  analytic=expected_tip([r[c] for c in ('Base_deg','Shoulder_deg','Elbow_deg')])
  assert math.dist(composed,analytic)<1e-10
print('PASS: URDF joint structure and independent transform-chain checks')
