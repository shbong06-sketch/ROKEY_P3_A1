#!/usr/bin/env python3
"""Build a fixed two-tine fork URDF and optionally attach it to M0609 tool0."""
import argparse
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def add_geometry(link, tag, name, xyz, size):
    element = ET.SubElement(link, tag, {"name": name})
    ET.SubElement(element, "origin", {"xyz": " ".join(f"{v:.6f}" for v in xyz), "rpy": "0 0 0"})
    geometry = ET.SubElement(element, "geometry")
    ET.SubElement(geometry, "box", {"size": " ".join(f"{v:.6f}" for v in size)})
    if tag == "visual":
        material = ET.SubElement(element, "material", {"name": "fork_blue"})
        ET.SubElement(material, "color", {"rgba": "0.16 0.38 0.68 1"})


def make_fork(width, spacing, length, thickness, plate_width, plate_height, plate_depth, mass):
    if min(width, spacing, length, thickness, plate_width, plate_height, plate_depth, mass) <= 0:
        raise ValueError("All dimensions and mass must be positive")
    if spacing <= width:
        raise ValueError("fork center spacing must exceed tine width")
    if plate_width < spacing + width:
        raise ValueError("plate width must cover both fork tines")
    link = ET.Element("link", {"name": "fork_tool"})
    # tool0 +Z points toward the fork tips. Pallet approach rotates the robot wrist
    # to put that axis horizontally into the rack. The plate sits in front of tool0.
    plate_center = plate_depth / 2
    tine_center = plate_depth + length / 2
    parts = [
        ("mount_plate", (0, 0, plate_center), (plate_width, plate_height, plate_depth)),
        ("left_tine", (-spacing / 2, 0, tine_center), (width, thickness, length)),
        ("right_tine", (spacing / 2, 0, tine_center), (width, thickness, length)),
    ]
    for name, position, size in parts:
        add_geometry(link, "visual", name, position, size)
        add_geometry(link, "collision", name, position, size)
    inertial = ET.SubElement(link, "inertial")
    # Conservative box approximation for the complete tool; tune from CAD if available.
    ET.SubElement(inertial, "origin", {"xyz": f"0 0 {plate_depth + length / 3:.6f}", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": f"{mass:.6f}"})
    sx, sy, sz = plate_width, max(plate_height, thickness), plate_depth + length
    ixx = mass * (sy * sy + sz * sz) / 12
    iyy = mass * (sx * sx + sz * sz) / 12
    izz = mass * (sx * sx + sy * sy) / 12
    ET.SubElement(inertial, "inertia", {"ixx": f"{ixx:.8f}", "ixy": "0", "ixz": "0", "iyy": f"{iyy:.8f}", "iyz": "0", "izz": f"{izz:.8f}"})
    return link, plate_depth + length


def add_mount(robot, link):
    existing_links = {node.attrib["name"] for node in robot.findall("link")}
    if "tool0" not in existing_links:
        raise ValueError("M0609 URDF does not have tool0; inspect the end link first")
    if "fork_tool" in existing_links:
        raise ValueError("fork_tool already exists")
    robot.append(copy.deepcopy(link))
    joint = ET.SubElement(robot, "joint", {"name": "tool0_to_fork_tool", "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": "tool0"})
    ET.SubElement(joint, "child", {"link": "fork_tool"})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})


def save(root, path):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--m0609-urdf", type=Path, help="Existing m0609_isaac_sim.urdf; writes merged M0609+fork URDF")
    p.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "output")
    p.add_argument("--tine-width", type=float, default=0.018)
    p.add_argument("--tine-spacing", type=float, default=0.090, help="Center to center")
    p.add_argument("--tine-length", type=float, default=0.220)
    p.add_argument("--tine-thickness", type=float, default=0.012)
    p.add_argument("--plate-width", type=float, default=0.140)
    p.add_argument("--plate-height", type=float, default=0.070)
    p.add_argument("--plate-depth", type=float, default=0.025)
    p.add_argument("--tool-mass", type=float, default=0.45, help="Simulation estimate in kg")
    args = p.parse_args()
    link, tip_z = make_fork(args.tine_width, args.tine_spacing, args.tine_length, args.tine_thickness, args.plate_width, args.plate_height, args.plate_depth, args.tool_mass)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    standalone = ET.Element("robot", {"name": "fixed_fork_tool"})
    standalone.append(copy.deepcopy(link))
    save(standalone, args.output_dir / "fork_tool.urdf")
    if args.m0609_urdf:
        source = args.m0609_urdf.expanduser().resolve()
        robot = ET.parse(source).getroot()
        add_mount(robot, link)
        # The original project URDF uses /home/rokey/... mesh paths. Resolve them
        # to the actual checkout so the generated file can be imported elsewhere.
        source_mesh_prefix = "/home/rokey/cobot3_ws/isaacpjt/M0609/doosan-robot2/urdf/meshes/"
        actual_mesh_dir = source.parent / "meshes"
        for mesh in robot.findall(".//mesh"):
            filename = mesh.get("filename", "")
            if filename.startswith(source_mesh_prefix):
                mesh.set("filename", str(actual_mesh_dir / filename[len(source_mesh_prefix):]))
        save(robot, args.output_dir / "m0609_with_fork.urdf")
    print(f"Fork tip in tool0 frame: (0, 0, {tip_z:.3f}) m")
    print(f"Output: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
