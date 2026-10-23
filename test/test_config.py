"""Unit tests validating YAML configs, URDF model, and SDF simulation world."""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import pytest
import yaml

pkg_root = Path(__file__).resolve().parent.parent


def test_slam_toolbox_config():
    cfg_file = pkg_root / "config" / "slam_toolbox_params.yaml"
    assert cfg_file.is_file(), f"Missing config file: {cfg_file}"

    with open(cfg_file) as f:
        data = yaml.safe_load(f)

    assert "slam_toolbox" in data
    params = data["slam_toolbox"]["ros__parameters"]
    assert params["odom_frame"] == "odom"
    assert params["map_frame"] == "map"
    assert params["base_frame"] == "base_footprint"
    assert params["scan_topic"] == "/scan"
    assert "resolution" in params
    assert params["resolution"] > 0


def test_nav2_config():
    cfg_file = pkg_root / "config" / "nav2_params.yaml"
    assert cfg_file.is_file(), f"Missing config file: {cfg_file}"

    with open(cfg_file) as f:
        data = yaml.safe_load(f)

    expected_nodes = [
        "amcl",
        "bt_navigator",
        "controller_server",
        "planner_server",
        "local_costmap",
        "global_costmap",
    ]
    for node in expected_nodes:
        assert node in data, f"Node {node} missing from nav2_params.yaml"

    # Verify controller uses DWB or appropriate plugin
    controller_params = data["controller_server"]["ros__parameters"]
    assert "FollowPath" in controller_params

    # Verify planner uses Navfn or GridBased planner
    planner_params = data["planner_server"]["ros__parameters"]
    assert "GridBased" in planner_params


def test_gazebo_bridge_config():
    cfg_file = pkg_root / "config" / "gazebo_bridge.yaml"
    assert cfg_file.is_file(), f"Missing config file: {cfg_file}"

    with open(cfg_file) as f:
        bridges = yaml.safe_load(f)

    assert isinstance(bridges, list)
    assert len(bridges) > 0

    topics = [b["ros_topic_name"] for b in bridges if "ros_topic_name" in b]
    assert "/cmd_vel" in topics
    assert "/scan" in topics
    assert "/wheel_odom" in topics or "/odom" in topics


def test_urdf_validity():
    urdf_file = pkg_root / "urdf" / "amr_nav.urdf"
    assert urdf_file.is_file(), f"Missing URDF file: {urdf_file}"

    tree = ET.parse(urdf_file)
    root = tree.getroot()
    assert root.tag == "robot"
    assert root.attrib.get("name") == "amr_nav"

    # Check links
    links = [elem.attrib["name"] for elem in root.findall("link")]
    assert "base_footprint" in links
    assert "base_link" in links
    assert "left_wheel" in links
    assert "right_wheel" in links
    assert "laser_link" in links

    # Check joints
    joints = [elem.attrib["name"] for elem in root.findall("joint")]
    assert "base_joint" in joints
    assert "left_wheel_joint" in joints
    assert "right_wheel_joint" in joints
    assert "laser_joint" in joints

    # Check sensor in gazebo tags
    gazebo_tags = root.findall("gazebo")
    assert len(gazebo_tags) > 0
    sensors = root.findall(".//sensor")
    assert len(sensors) > 0
    sensor_types = [s.attrib.get("type") for s in sensors]
    assert "gpu_lidar" in sensor_types or "ray" in sensor_types or "lidar" in sensor_types


def test_world_sdf_validity():
    sdf_file = pkg_root / "worlds" / "warehouse_nav.sdf"
    assert sdf_file.is_file(), f"Missing SDF file: {sdf_file}"

    tree = ET.parse(sdf_file)
    root = tree.getroot()
    assert root.tag == "sdf"
    world = root.find("world")
    assert world is not None
    assert world.attrib.get("name") == "warehouse_nav"

    models = [m.attrib.get("name") for m in world.findall("model")]
    assert "wall_north" in models
    assert "wall_south" in models
    assert "shelf_1" in models
    assert "pillar_1" in models


def test_default_map():
    yaml_file = pkg_root / "maps" / "warehouse_default.yaml"
    assert yaml_file.is_file(), f"Missing map YAML: {yaml_file}"

    with open(yaml_file) as f:
        meta = yaml.safe_load(f)

    assert "image" in meta
    assert "resolution" in meta
    assert meta["resolution"] > 0
    assert "origin" in meta
    assert len(meta["origin"]) == 3

    pgm_file = pkg_root / "maps" / meta["image"]
    assert pgm_file.is_file(), f"Map image missing: {pgm_file}"
    assert pgm_file.stat().st_size > 0
