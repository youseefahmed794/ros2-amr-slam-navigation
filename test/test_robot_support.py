
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

def transform(element):
    matrix = np.eye(4)
    if element is None:
        return matrix
    matrix[:3, 3] = list(map(float, element.get("xyz", "0 0 0").split()))
    roll, pitch, yaw = map(float, element.get("rpy", "0 0 0").split())
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    matrix[:3, :3] = rz @ ry @ rx
    return matrix

def test_robot_com_inside_nominal_support_triangle():
    path = Path(__file__).resolve().parents[1] / "urdf/amr_nav.urdf"
    robot = ET.parse(path).getroot()
    links = {link.get("name"): link for link in robot.findall("link")}
    frames = {"base_footprint": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent in frames:
                frames[child] = frames[parent] @ transform(joint.find("origin"))
                pending.remove(joint)
                progressed = True
        assert progressed, "Unresolved URDF link transforms"

    total_mass = 0.0
    moment = np.zeros(3)
    for name, link in links.items():
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        assert mass > 0
        center = (frames[name] @ transform(inertial.find("origin")))[:3, 3]
        total_mass += mass
        moment += mass * center
    com = moment / total_mass

    supports = []
    heights = []
    for name in ("left_wheel", "right_wheel", "caster"):
        collision = links[name].find("collision")
        matrix = frames[name] @ transform(collision.find("origin"))
        cylinder = collision.find("geometry/cylinder")
        sphere = collision.find("geometry/sphere")
        if cylinder is not None:
            assert abs(matrix[2, 2]) < 1e-6, "Wheel axis must be horizontal"
            radius = float(cylinder.get("radius"))
        else:
            assert sphere is not None, "Unsupported support geometry"
            radius = float(sphere.get("radius"))
        supports.append(matrix[:2, 3])
        heights.append(matrix[2, 3] - radius)

    assert max(heights) - min(heights) < 1e-6
    triangle = np.array(supports)
    weights = np.linalg.solve(
        np.vstack((triangle.T, np.ones(3))),
        np.r_[com[:2], 1.0]
    )
    assert np.all(weights > 0), (
        f"COM outside nominal support triangle: com={com}, weights={weights}"
    )
    margins = []
    for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
        edge = b - a
        offset = com[:2] - a
        cross = edge[0]*offset[1] - edge[1]*offset[0]
        margins.append(abs(cross) / np.linalg.norm(edge))
    assert min(margins) > 0.01, f"Insufficient nominal static margin: {margins}"
