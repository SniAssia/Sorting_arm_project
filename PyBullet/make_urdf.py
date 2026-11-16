"""Generate a *parametric proxy* of SEVEN's 3-DOF structure, not a CAD assembly.

SI units (m, rad). Kinematics EXACTLY match team's custom Arduino FK:
  h = L1*cos(s) + L2*cos(s+e-pi)
  x = h*sin(b), y = h*cos(b), z = L1*sin(s)+L2*sin(s+e-pi)
Bullet yaw = -b, shoulder pitch = +s, elbow pitch = e-pi.
World Z is 0.06m above the MATLAB shoulder-origin frame.
"""
from __future__ import annotations
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom.minidom import parseString

ROOT = Path(__file__).resolve().parent
L1 = 0.12
L2 = 0.12
SHOULDER_HEIGHT = 0.06


def origin(parent, xyz="0 0 0", rpy="0 0 0"):
    return ET.SubElement(parent, "origin", xyz=xyz, rpy=rpy)


def make_link(robot, name, color="0.25 0.5 0.75 1", shape="box", size="0.02 0.05 0.02",
              center="0 0 0", mass=0.08, visual=True):
    node = ET.SubElement(robot, "link", name=name)
    inertial = ET.SubElement(node, "inertial")
    origin(inertial)
    ET.SubElement(inertial,"mass",value=str(mass))
    ET.SubElement(inertial,"inertia",ixx="0.0001",ixy="0",ixz="0",iyy="0.0001",iyz="0",izz="0.0001")
    if visual:
        v=ET.SubElement(node,"visual")
        origin(v,xyz=center)
        g=ET.SubElement(v,"geometry")
        if shape == "box":
            ET.SubElement(g,"box",size=size)
        elif shape == "cylinder":
            radius,length=size.split()
            ET.SubElement(g,"cylinder",radius=radius,length=length)
        else:
            ET.SubElement(g,"sphere",radius=size)
        m=ET.SubElement(v,"material",name=name+"_material")
        ET.SubElement(m,"color",rgba=color)
    return node


def make_joint(robot,name,typ,parent,child,xyz="0 0 0",axis="0 0 1",lower="-3.1415926535",upper="3.1415926535"):
    j=ET.SubElement(robot,"joint",name=name,type=typ)
    ET.SubElement(j,"parent",link=parent)
    ET.SubElement(j,"child",link=child)
    origin(j,xyz=xyz)
    if typ!="fixed":
        ET.SubElement(j,"axis",xyz=axis)
        ET.SubElement(j,"limit",lower=lower,upper=upper,effort="20",velocity="8")
    return j


def generate(path: Path = ROOT/"robot"/"seven_kinematic.urdf") -> Path:
    r=ET.Element("robot",name="SEVEN_3DOF_parametric_proxy")
    make_link(r,"base_link",color="0.16 0.20 0.26 1",shape="cylinder",size="0.027 0.06",center="0 0 0.03",mass=1.0)
    make_link(r,"yaw_link",color="0.20 0.28 0.37 1",shape="cylinder",size="0.029 0.018",mass=.15)
    make_link(r,"shoulder_link",color="0.22 0.47 0.77 1",size=f"0.022 {L1:.6f} 0.021",center=f"0 {L1/2:.6f} 0",mass=.14)
    make_link(r,"elbow_link",color="0.25 0.62 0.79 1",size=f"0.019 {L2:.6f} 0.017",center=f"0 {L2/2:.6f} 0",mass=.1)
    make_link(r,"gripper_link",color="0.14 0.21 0.28 1",size="0.048 0.026 0.023",center="0 -0.015 0",mass=.06)
    make_link(r,"left_jaw",color="0.95 0.62 0.23 1",size="0.006 0.035 0.012",center="0 0.010 0",mass=.015)
    make_link(r,"right_jaw",color="0.95 0.62 0.23 1",size="0.006 0.035 0.012",center="0 0.010 0",mass=.015)
    make_link(r,"tool_marker",color="0.85 0.15 0.10 1",shape="sphere",size="0.003",mass=.002)
    make_joint(r,"base_yaw","revolute","base_link","yaw_link",xyz=f"0 0 {SHOULDER_HEIGHT:.6f}",axis="0 0 1")
    make_joint(r,"shoulder_pitch","revolute","yaw_link","shoulder_link",axis="1 0 0",lower="0",upper="3.1415926535")
    make_joint(r,"elbow_pitch","revolute","shoulder_link","elbow_link",xyz=f"0 {L1:.6f} 0",axis="1 0 0",lower="-3.1415926535",upper="0")
    make_joint(r,"wrist_mount","fixed","elbow_link","gripper_link",xyz=f"0 {L2:.6f} 0")
    make_joint(r,"left_grip","prismatic","gripper_link","left_jaw",xyz="0 -0.004 0",axis="1 0 0",lower="0.006",upper="0.026")
    make_joint(r,"right_grip","prismatic","gripper_link","right_jaw",xyz="0 -0.004 0",axis="1 0 0",lower="-0.026",upper="-0.006")
    make_joint(r,"tool_fixed","fixed","gripper_link","tool_marker")
    # No collision geometry for moving bodies: scripted kinematic playback,
    # explicitly NOT a collision/contact-accurate grasp dynamics simulation.
    xml=parseString(ET.tostring(r,encoding="utf-8")).toprettyxml(indent="  ")
    path.parent.mkdir(exist_ok=True,parents=True)
    path.write_text(xml,encoding="utf-8")
    return path


if __name__=="__main__":
    print("Created",generate())
