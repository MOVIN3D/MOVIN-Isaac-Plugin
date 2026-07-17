"""
Generate a MuJoCo MJCF skeleton from a MOVIN T-pose BVH file.

Parses the hierarchy (joints + OFFSETs + End Sites) of a T-pose BVH and emits an
MJCF whose structure and style mirror ``data/movinman_skeleton.xml``: the same
``<default>`` classes, one free joint at the root and three orthogonal hinges per
non-root body, and per-body geoms mirrored (by name) from the legacy skeleton.

Coordinate convention: BVH offsets are Y-up meters; MJCF is Z-up.  A point is
mapped ``(x, y, z)_yup -> (x, -z, y)_zup`` (matching ``yup_to_zup_vec``).

Geoms are keyed by body NAME against a reference MJCF (default
``data/movinman_skeleton.xml``): a body that exists in the reference reuses its
geom type/size/density, and capsule ``fromto`` endpoints are recomputed toward the
body's primary child so the proxy matches the T-pose being generated.  Bodies that
do not exist in the reference fall back to a template of a related body
(Spine2/Spine3 -> Spine1, Neck1 -> Neck).  Actuator gears are mirrored the same way.

Usage:
    python scripts/generate_skeleton_mjcf.py <tpose.bvh> <out.xml> [--model-name NAME]

Example:
    python scripts/generate_skeleton_mjcf.py data/MOVINManV3_Tpose.bvh \
        data/movinman_v3_skeleton.xml --model-name movinman_v3
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET


# Default reference MJCF whose geom/gear styling is mirrored (plugin data dir).
DEFAULT_REFERENCE_MJCF = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "data", "movinman_skeleton.xml"
)

# Section comments emitted just before the named body opens (mirrors the legacy
# file's hand-authored comments).  Bodies not listed here emit no comment.
SECTION_COMMENTS = {
    "Spine": "Spine chain",
    "Neck": "Neck/Head",
    "RightShoulder": "Right arm chain",
    "RightHandIndex1": "Right hand fingers",
    "LeftShoulder": "Left arm chain",
    "LeftHandIndex1": "Left hand fingers",
    "RightUpLeg": "Right leg chain",
    "LeftUpLeg": "Left leg chain",
}

# Bodies missing from the reference borrow a related body's geom/gear template.
TEMPLATE_FALLBACK = {
    "Spine2": "Spine1",
    "Spine3": "Spine1",
    "Neck1": "Neck",
}

# Per-axis hinge definitions applied to every non-root body.
HINGE_AXES = (("x", "1 0 0"), ("y", "0 1 0"), ("z", "0 0 1"))
HINGE_RANGE = "-3.14159 3.14159"


class BvhNode:
    """A single node of a parsed BVH hierarchy (joint or End Site)."""

    def __init__(self, name, offset, is_end_site=False):
        self.name = name              # joint name, or None for an End Site
        self.offset = offset          # (x, y, z) in BVH Y-up meters
        self.is_end_site = is_end_site
        self.children = []


def parse_bvh_hierarchy(path):
    """
    Parse the HIERARCHY section of a BVH file, including End Sites.

    ``read_bvh`` in the SDK discards End Sites, so this tiny parser reads them
    itself (leaf bodies use the End Site offset to orient their geom).

    Args:
        path: BVH filename.

    Returns:
        The root BvhNode of the hierarchy.
    """
    root = None
    stack = []
    with open(path, "r") as f:
        for raw in f:
            tokens = raw.split()
            if not tokens:
                continue
            key = tokens[0]
            if key in ("ROOT", "JOINT"):
                node = BvhNode(tokens[1], (0.0, 0.0, 0.0))
                if stack:
                    stack[-1].children.append(node)
                else:
                    root = node
                stack.append(node)
            elif key == "End" and len(tokens) > 1 and tokens[1] == "Site":
                node = BvhNode(None, (0.0, 0.0, 0.0), is_end_site=True)
                stack[-1].children.append(node)
                stack.append(node)
            elif key == "OFFSET":
                x, y, z = (float(v) for v in tokens[1:4])
                stack[-1].offset = (x, y, z)
            elif key == "}":
                stack.pop()
            elif key == "MOTION":
                break
            # CHANNELS and "{" are ignored.
    if root is None:
        raise ValueError("No ROOT joint found in BVH hierarchy: %s" % path)
    return root


def yup_to_zup(offset):
    """Convert a Y-up BVH offset (x, y, z) to a Z-up MJCF position (x, -z, y)."""
    x, y, z = offset
    return (x, -z, y)


def fmt(v):
    """Format a float for MJCF: up to 6 decimals, trailing zeros stripped, no -0."""
    v = float(v)
    if abs(v) < 1e-9:
        return "0"
    s = "{:.6f}".format(v).rstrip("0").rstrip(".")
    if s in ("", "-0"):
        return "0"
    return s


def fmt_vec(vec):
    """Format a 3-vector as a space-separated MJCF attribute string."""
    return " ".join(fmt(c) for c in vec)


def dfs_bodies(node):
    """Yield joint nodes (skipping End Sites) in depth-first order."""
    if node.is_end_site:
        return
    yield node
    for child in node.children:
        yield from dfs_bodies(child)


def compute_hips_height(root):
    """
    Compute the root height as ``max(0, -min cumulative Y)`` over all joints.

    With the T-pose's zero rotations, world position is just the running sum of
    OFFSETs, so the lowest joint (feet) sets how high the hips sit above ground.
    """
    min_y = [0.0]

    def walk(node, cum_y):
        if node.is_end_site:
            return
        cum_y += node.offset[1]
        min_y[0] = min(min_y[0], cum_y)
        for child in node.children:
            walk(child, cum_y)

    walk(root, 0.0)
    return round(max(0.0, -min_y[0]), 4)


def primary_child(node):
    """
    Pick the body child a capsule geom should point toward.

    Single-child bodies use that child; multi-child bodies pick the anatomical
    continuation (spine -> Neck/next Spine, Hand -> Middle1).  Leaf bodies return
    ``None`` (the caller falls back to the End Site offset direction).
    """
    body_children = [c for c in node.children if not c.is_end_site]
    if not body_children:
        return None
    if len(body_children) == 1:
        return body_children[0]
    if node.name.endswith("Hand"):
        for c in body_children:
            if c.name.endswith("Middle1"):
                return c
    for c in body_children:
        if c.name == "Neck":
            return c
    for c in body_children:
        if c.name.startswith("Spine"):
            return c
    return body_children[0]


def capsule_endpoint(node):
    """Local MJCF endpoint a capsule reaches toward: primary child, else End Site."""
    child = primary_child(node)
    if child is not None:
        endpoint = yup_to_zup(child.offset)
    else:
        end_sites = [c for c in node.children if c.is_end_site]
        if end_sites:
            endpoint = yup_to_zup(end_sites[0].offset)
        else:
            endpoint = (0.0, 0.0, 0.05)
    if all(abs(c) < 1e-9 for c in endpoint):
        endpoint = (0.0, 0.0, 0.05)
    return endpoint


def load_reference(reference_path):
    """
    Extract geom templates, childclass tags and actuator gears from a reference MJCF.

    Returns:
        (geoms, childclass, gears): dicts keyed by body name.  ``geoms`` maps to the
        geom element's attribute dict, ``childclass`` to the body's childclass (or
        None), and ``gears`` to the actuator gear string shared by the body's hinges.
    """
    tree = ET.parse(reference_path)
    root = tree.getroot()

    geoms = {}
    childclass = {}
    for body in root.iter("body"):
        name = body.get("name")
        childclass[name] = body.get("childclass")
        geom = body.find("geom")
        if geom is not None:
            geoms[name] = dict(geom.attrib)

    gears = {}
    actuator = root.find("actuator")
    if actuator is not None:
        for motor in actuator.iter("motor"):
            joint = motor.get("joint")
            body_name = joint.rsplit("_", 1)[0]  # strip the _x/_y/_z axis suffix
            gears[body_name] = motor.get("gear")

    return geoms, childclass, gears


def resolve_template(name, table, kind):
    """Look up a body's template value, falling back to a related body if missing."""
    if name in table:
        return table[name]
    fallback = TEMPLATE_FALLBACK.get(name)
    if fallback is not None and fallback in table:
        return table[fallback]
    return None


def build_geom_xml(node, geoms):
    """Build the ``<geom>`` line for a body, mirroring the reference by name."""
    name = node.name
    template = resolve_template(name, geoms, "geom")
    if template is None:
        sys.stderr.write(
            "[warn] no geom template for body '%s'; using default sphere\n" % name
        )
        return '<geom name="%s" type="sphere" size=".05" density="1000"/>' % name

    gtype = template.get("type", "sphere")
    density = template.get("density", "1000")
    size = template.get("size", ".05")

    if gtype == "capsule":
        fromto = "0 0 0 " + fmt_vec(capsule_endpoint(node))
        return ('<geom name="%s" type="capsule" fromto="%s" size="%s" density="%s"/>'
                % (name, fromto, size, density))
    if gtype == "box":
        pos = template.get("pos", "0 0 0")
        return ('<geom name="%s" type="box" pos="%s" size="%s" density="%s"/>'
                % (name, pos, size, density))
    # sphere (or anything else): reuse pos only if the reference set one.
    pos = template.get("pos")
    pos_attr = ' pos="%s"' % pos if pos is not None else ""
    return ('<geom name="%s" type="%s"%s size="%s" density="%s"/>'
            % (name, gtype, pos_attr, size, density))


def emit_body(node, depth, hips_height, geoms, childclass, lines):
    """Recursively append the MJCF lines for a body and its descendants."""
    indent = "  " * depth
    name = node.name

    if name in SECTION_COMMENTS:
        lines.append("%s<!-- %s -->" % (indent, SECTION_COMMENTS[name]))

    is_root = depth == 2  # worldbody is at depth 1, the root body at depth 2.
    if is_root:
        lines.append('%s<body name="%s" pos="0 0 %s" childclass="body">'
                     % (indent, name, fmt(hips_height)))
        lines.append('%s  <freejoint name="root"/>' % indent)
    else:
        cc = resolve_template(name, childclass, "childclass")
        cc_attr = ' childclass="%s"' % cc if cc else ""
        pos = fmt_vec(yup_to_zup(node.offset))
        lines.append('%s<body name="%s" pos="%s"%s>' % (indent, name, pos, cc_attr))
        for axis, vec in HINGE_AXES:
            lines.append('%s  <joint name="%s_%s" type="hinge" axis="%s" range="%s"/>'
                         % (indent, name, axis, vec, HINGE_RANGE))

    lines.append("%s  %s" % (indent, build_geom_xml(node, geoms)))

    for child in node.children:
        if child.is_end_site:
            continue
        lines.append("")
        emit_body(child, depth + 1, hips_height, geoms, childclass, lines)

    lines.append("%s</body>" % indent)


def build_actuators(root, gears):
    """Build the ``<actuator>`` block: one motor per hinge in DFS body order."""
    lines = ["  <actuator>"]
    for node in dfs_bodies(root):
        if node is root:
            continue
        gear = resolve_template(node.name, gears, "gear")
        if gear is None:
            sys.stderr.write(
                "[warn] no gear for body '%s'; using default 50\n" % node.name
            )
            gear = "50"
        for axis, _ in HINGE_AXES:
            jname = "%s_%s" % (node.name, axis)
            lines.append('    <motor name="%s" gear="%s" joint="%s"/>'
                         % (jname, gear, jname))
    lines.append("  </actuator>")
    return lines


def generate_mjcf(bvh_path, model_name, reference_path):
    """Parse a T-pose BVH and return the full MJCF document as a string."""
    root = parse_bvh_hierarchy(bvh_path)
    hips_height = compute_hips_height(root)
    geoms, childclass, gears = load_reference(reference_path)

    lines = ['<mujoco model="%s">' % model_name]
    lines.append('  <compiler angle="radian"/>')
    lines.append("")
    lines.append("  <default>")
    lines.append('    <motor ctrlrange="-1 1" ctrllimited="true"/>')
    lines.append('    <default class="body">')
    lines.append('      <geom condim="1" friction="1.0 0.05 0.05" solimp=".9 .99 .003" solref=".015 1"/>')
    lines.append('      <joint limited="true" solimplimit="0 .99 .01" armature=".01"/>')
    lines.append("    </default>")
    lines.append('    <default class="hand">')
    lines.append('      <geom condim="1" friction="1.0 0.05 0.05" solimp=".9 .99 .003" solref=".015 1"/>')
    lines.append('      <joint limited="true" solimplimit="0 .99 .01" armature=".005"/>')
    lines.append("    </default>")
    lines.append("  </default>")
    lines.append("")
    lines.append("  <worldbody>")

    emit_body(root, 2, hips_height, geoms, childclass, lines)

    lines.append("  </worldbody>")
    lines.append("")
    lines.extend(build_actuators(root, gears))
    lines.append("</mujoco>")
    lines.append("")

    body_count = sum(1 for _ in dfs_bodies(root))
    print("[info] model '%s': %d bodies, %d hinge joints, hips_height=%s"
          % (model_name, body_count, (body_count - 1) * 3, fmt(hips_height)))

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Generate a MuJoCo MJCF skeleton from a MOVIN T-pose BVH file."
    )
    parser.add_argument("bvh", help="Path to the T-pose BVH file")
    parser.add_argument("out", help="Path to write the generated MJCF")
    parser.add_argument("--model-name", default=None,
                        help="MuJoCo model name (default: output filename stem)")
    parser.add_argument("--reference-mjcf", default=DEFAULT_REFERENCE_MJCF,
                        help="Reference MJCF whose geom/gear styling is mirrored")
    args = parser.parse_args()

    model_name = args.model_name or os.path.splitext(os.path.basename(args.out))[0]
    xml_str = generate_mjcf(args.bvh, model_name, args.reference_mjcf)

    with open(args.out, "w") as f:
        f.write(xml_str)
    print("[info] wrote %s" % args.out)


if __name__ == "__main__":
    main()
