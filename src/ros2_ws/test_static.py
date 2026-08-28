#!/usr/bin/env python3
from __future__ import annotations

import ast
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent


def main() -> None:
    python_files = sorted(ROOT.glob("src/**/*.py"))
    for path in python_files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    xml_files = sorted(ROOT.glob("src/**/package.xml"))
    xml_files += sorted(ROOT.glob("src/**/*.srdf"))
    xml_files += sorted(ROOT.glob("src/**/*.xacro"))
    for path in xml_files:
        ET.parse(path)

    driver = (
        ROOT
        / "src/eee8097_roarm_driver/eee8097_roarm_driver/roarm_driver_node.py"
    ).read_text(encoding="utf-8")
    controller = (
        ROOT / "src/eee8097_moveit_config/config/moveit_controllers.yaml"
    ).read_text(encoding="utf-8")
    srdf = (
        ROOT / "src/eee8097_moveit_config/config/roarm_m2_safe.srdf"
    ).read_text(encoding="utf-8")

    forbidden_driver_tokens = (
        "set_gripper_radians(",
        "move_all_joints_degrees(",
        'joint=4',
        '"T": 106',
        "/gripper_cmd",
    )
    for token in forbidden_driver_tokens:
        assert token not in driver, f"unsafe driver token present: {token}"
    assert "gripper_controller" not in controller
    assert "link3_to_gripper_link" not in controller
    assert '<group name="gripper">' not in srdf
    assert '<end_effector ' not in srdf
    print(
        f"Static ROS 2 checks passed: {len(python_files)} Python files, "
        f"{len(xml_files)} XML files; gripper command path absent."
    )


if __name__ == "__main__":
    main()
