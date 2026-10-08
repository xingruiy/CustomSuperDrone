"""Livox Mid-360S LiDAR, as a STEP assembly.

The model has the outer shape of the sensor: base cover with mounting holes,
grooved body, M12 connector, knurled ring and dome window. It can also add the
field of view (FOV) as a solid, for checks that the view is clear. Sizes come
from the Livox Mid-360S user manual (Dimensions section) and the official
Livox Mid-360S STEP model.

Coordinates, all in mm:
    Z: up. Z = 0 is the bottom (mounting) face. The origin is the center of it.
    X: +X is the M12 connector side.
    The optical center (the LiDAR origin) is at (0, 0, 47).
With --origin optical, the model moves so the optical center is at the origin.

Usage:
    uv run python mid360s.py
    uv run python mid360s.py --origin optical -o mid360s_optical.step
    uv run python mid360s.py --fov-range 300 -o mid360s_fov.step
"""

import argparse
import dataclasses
import math
from dataclasses import dataclass, field

import cadquery as cq

OPTICAL_Z = 47.0  # optical center height above the bottom face
FOV_MIN, FOV_MAX = -7.0, 52.0  # vertical FOV in degrees (horizontal FOV is 360)

# Base cover and body
BASE_W = 64.8
BASE_T = 3.87
BODY_W = 64.88  # width at the bottom of the body
BODY_TAPER_Z = 23.33  # the side walls lean inward up to here
BODY_TOP_W = 63.52
BODY_TOP = 28.5
CORNER_R = 4.5
TOP_ROUND = 4.0
# Neck, ring and dome: (radius, z) points of the turned profiles
NECK_PROFILE = [(0.0, 28.0), (26.8, 28.0), (25.64, 30.36), (25.53, 32.52), (25.0, 33.02), (0.0, 33.02)]
RING_PROFILE = [(22.3, 33.02), (25.0, 33.02), (25.3, 33.62), (25.3, 38.64), (24.0, 39.5),
                (22.6, 39.5), (22.3, 39.2)]
DOME_R = 22.0
DOME_CENTER_Z = 39.3
TOP_Z = 60.0
# Grooves on the +Y and -Y faces: the bottom of each groove is a slope, as (y, z) points.
GROOVE_TOP = 27.9
GROOVE_FLOOR = [(18.2, GROOVE_TOP), (18.2, 16.7), (20.4, 16.6), (33.5, 3.6)]
# Mounting
M3_HOLES = [(x, y) for x in (-24.0, 24.0) for y in (-18.0, 18.0)]
M3_DEPTH = 5.0
LOCATE_ROUND = (0.0, 16.0)
LOCATE_SLOT = (0.0, -23.25)
LOCATE_D, LOCATE_DEPTH = 3.0, 1.8
CORNER_SCREWS = [(x, y) for x in (-29.2, 29.2) for y in (-27.5, 27.5)]
# M12 connector
CONN_Z = 17.3
POCKET = (26.8, 11.3, 8.8, 25.5)  # floor x, half width, z0, z1
FLANGE = (26.8, 29.0, 10.9, 9.2, 25.2)  # x0, x1, half width, z0, z1
FLANGE_SCREWS = [(y, z) for y in (-8.0, 8.0) for z in (12.25, 22.25)]

BODY_GRAY = (0.17, 0.17, 0.19)
BASE_GRAY = (0.12, 0.12, 0.13)
RING_BLACK = (0.07, 0.07, 0.08)
WINDOW = (0.10, 0.12, 0.17)
METAL = (0.75, 0.75, 0.78)
SCREW = (0.35, 0.35, 0.37)
INSERT_BLACK = (0.05, 0.05, 0.05)
GOLD = (0.85, 0.70, 0.25)
FOV_BLUE = (0.25, 0.60, 1.00, 0.2)


@dataclass
class Mid360Params:
    # Body grooves (on the +Y and -Y faces)
    groove_count: int = 9
    groove_pitch: float = 6.8
    groove_w: float = 4.4
    # Ring
    knurl_count: int = 40  # grooves around the ring (0 = smooth)
    # Parts to include
    connector: bool = True  # M12 connector and its flange
    mount_holes: bool = True  # M3 holes, locating holes and corner screws in the base
    fov_range: float = 0.0  # add the FOV as a solid out to this distance (0 = none)
    # Frame
    origin: str = field(default="base", metadata={"choices": ("base", "optical")})

    def __post_init__(self):
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if f.type in (int, float) and value < 0:
                raise ValueError(f"{f.name} must not be negative, got {value}")
            choices = f.metadata.get("choices")
            if choices and value not in choices:
                raise ValueError(f"{f.name} must be one of {choices}, got {value!r}")
        if self.groove_count > 0:
            if not 1.0 <= self.groove_w <= self.groove_pitch - 1.0:
                raise ValueError("groove_w must be at least 1 mm and leave a 1 mm rib (groove_pitch - 1)")
            half_span = (self.groove_count - 1) * self.groove_pitch / 2 + self.groove_w / 2
            if half_span > 29.5:
                raise ValueError(f"the grooves span {2 * half_span:.1f} mm, the face allows 59 mm")
        if 0 < self.knurl_count < 8:
            raise ValueError("knurl_count must be 0 or at least 8")
        if 0 < self.fov_range <= 30.0:
            raise ValueError("fov_range must be 0 or more than 30 mm")


# ---------------------------------------------------------------- helpers


def cyl(x, y, z0, z1, d) -> cq.Solid:
    return cq.Solid.makeCylinder(d / 2, z1 - z0, cq.Vector(x, y, z0))


def x_cyl(x0, x1, d, y=0.0, z=CONN_Z) -> cq.Solid:
    """Cylinder along X."""
    return cq.Solid.makeCylinder(d / 2, x1 - x0, cq.Vector(x0, y, z), cq.Vector(1, 0, 0))


def revolve(profile) -> cq.Solid:
    """Turn an (r, z) profile around the Z axis."""
    return cq.Workplane("XZ").polyline(profile).close().revolve(360, (0, 0, 0), (0, 1, 0)).val()


def fuse(solids) -> cq.Shape:
    first, *rest = solids
    return first.fuse(*rest).clean() if rest else first


# ---------------------------------------------------------------- parts


def mount_cutters(p: Mid360Params) -> list[cq.Solid]:
    """Holes in the bottom face. They go through the base cover into the body."""
    if not p.mount_holes:
        return []
    tools = [cyl(x, y, -1, M3_DEPTH, 3.0) for x, y in M3_HOLES]
    tools.append(cyl(*LOCATE_ROUND, -1, LOCATE_DEPTH, LOCATE_D))
    tools.append(cq.Workplane("XY", origin=(0, 0, -1)).center(*LOCATE_SLOT)
                 .slot2D(5.5, LOCATE_D, 90).extrude(LOCATE_DEPTH + 1).val())
    for x, y in CORNER_SCREWS:
        tools += [cyl(x, y, -1, 1.8, 4.4), cyl(x, y, 1.8, 7.3, 2.0)]
    return tools


def make_base(p: Mid360Params) -> cq.Shape:
    base = (cq.Workplane("XY").rect(BASE_W, BASE_W).extrude(BASE_T)
            .edges("|Z").fillet(CORNER_R).faces("<Z").chamfer(0.3).val())
    tools = mount_cutters(p)
    return base.cut(*tools).clean() if tools else base


def groove_cutters(p: Mid360Params) -> list[cq.Shape]:
    if p.groove_count == 0:
        return []
    # The side view of each groove: a vertical slot with a round top.
    xs = [(i - (p.groove_count - 1) / 2) * p.groove_pitch for i in range(p.groove_count)]
    slot_h = GROOVE_TOP + 5.0
    slots = [cq.Workplane("XZ", origin=(x, 45.0, GROOVE_TOP - slot_h / 2))
             .slot2D(slot_h, p.groove_w, 90).extrude(30.0).val() for x in xs]
    # The groove floor: deep at the top, it runs out to the face near the bottom.
    floor = (cq.Workplane("YZ", origin=(-40.0, 0, 0))
             .polyline(GROOVE_FLOOR + [(45.0, 3.6), (45.0, GROOVE_TOP)]).close().extrude(80.0).val())
    plus_y = cq.Compound.makeCompound(slots).intersect(floor)
    return [plus_y, plus_y.mirror("XZ")]


def make_body(p: Mid360Params) -> cq.Shape:
    taper = math.degrees(math.atan((BODY_W - BODY_TOP_W) / 2 / (BODY_TAPER_Z - BASE_T)))
    block = (cq.Workplane("XY", origin=(0, 0, BASE_T)).rect(BODY_W, BODY_W)
             .extrude(BODY_TOP - BASE_T, taper=taper)
             .edges("not(|X or |Y)").fillet(CORNER_R)
             .faces(">Z").edges().fillet(TOP_ROUND).val())
    body = block.fuse(revolve(NECK_PROFILE)).clean()
    tools = groove_cutters(p) + mount_cutters(p)
    if p.connector:
        fx, half_w, z0, z1 = POCKET
        tools.append(cq.Workplane("YZ", origin=(fx, 0, (z0 + z1) / 2)).rect(2 * half_w, z1 - z0)
                     .extrude(15.0).edges("|X").fillet(1.0).val())
    return body.cut(*tools).clean() if tools else body


def make_ring(p: Mid360Params) -> cq.Shape:
    ring = revolve(RING_PROFILE)
    if p.knurl_count == 0:
        return ring
    notch = cq.Solid.makeBox(0.8, 0.8, 4.6, cq.Vector(25.3 - 0.4, -0.4, 33.9))
    notches = [notch.rotate(cq.Vector(), cq.Vector(0, 0, 1), 360.0 * k / p.knurl_count)
               for k in range(p.knurl_count)]
    return ring.cut(*notches).clean()


def make_dome() -> cq.Shape:
    """Dome window: a sphere part with a flat top, on a short cylinder."""
    z0 = NECK_PROFILE[-1][1]
    sphere = cq.Solid.makeSphere(DOME_R, cq.Vector(0, 0, DOME_CENTER_Z), angleDegrees1=0, angleDegrees2=90)
    dome = fuse([sphere, cyl(0, 0, z0, DOME_CENTER_Z, 2 * DOME_R)])
    return dome.cut(cyl(0, 0, TOP_Z, TOP_Z + 5, 60.0), cyl(0, 0, TOP_Z - 0.4, TOP_Z + 1, 4.0)).clean()


def make_connector() -> dict[str, list]:
    """M12 A-coded 12-pin male connector on a square flange."""
    x0, x1, half_w, z0, z1 = FLANGE
    flange = cq.Solid.makeBox(x1 - x0, 2 * half_w, z1 - z0, cq.Vector(x0, -half_w, z0))
    shell = fuse([flange, x_cyl(x1, 31.0, 10.6), x_cyl(31.0, 36.7, 12.0),
                  x_cyl(36.7, 37.3, 10.8), x_cyl(37.3, 38.3, 10.2)])
    shell = shell.cut(x_cyl(28.3, 38.5, 8.2)).clean()
    insert = x_cyl(28.3, 37.0, 6.0)
    pin_pos = [(0.0, 0.0)] + [(2.2 * math.cos(math.radians(a)), 2.2 * math.sin(math.radians(a)))
                              for a in range(0, 360, 360 // 11)][:11]
    pins = [x_cyl(37.0, 38.0, 0.8, y, CONN_Z + z) for y, z in pin_pos]
    screws = [x_cyl(x1, x1 + 0.4, 3.6, y, z) for y, z in FLANGE_SCREWS]
    return {"connector": [shell], "connector_insert": [insert], "pins": pins, "screws": screws}


def make_corner_screws() -> list[cq.Solid]:
    """Base cover screws: heads in the corner counterbores, shanks in the holes."""
    screws = []
    for x, y in CORNER_SCREWS:
        head = fuse([cyl(x, y, 0.3, 1.8, 4.0), cyl(x, y, 1.8, 7.3, 2.0)])
        socket = cq.Workplane("XY", origin=(x, y, 0.2)).polygon(6, 2.2).extrude(0.9).val()
        screws.append(head.cut(socket).clean())
    return screws


def make_fov(p: Mid360Params) -> cq.Solid:
    """FOV from 25 mm (outside the sensor) to fov_range from the optical center."""
    def pt(r, deg):
        return (r * math.cos(math.radians(deg)), OPTICAL_Z + r * math.sin(math.radians(deg)))
    r0, r1 = 25.0, p.fov_range
    mid = (FOV_MIN + FOV_MAX) / 2
    profile = (cq.Workplane("XZ").moveTo(*pt(r0, FOV_MIN)).lineTo(*pt(r1, FOV_MIN))
               .threePointArc(pt(r1, mid), pt(r1, FOV_MAX)).lineTo(*pt(r0, FOV_MAX))
               .threePointArc(pt(r0, mid), pt(r0, FOV_MIN)).close())
    return profile.revolve(360, (0, 0, 0), (0, 1, 0)).val()


# ---------------------------------------------------------------- assembly


def build_mid360s(p: Mid360Params) -> cq.Assembly:
    parts: list[tuple[str, list, tuple]] = [
        ("base", [make_base(p)], BASE_GRAY),
        ("body", [make_body(p)], BODY_GRAY),
        ("ring", [make_ring(p)], RING_BLACK),
        ("window", [make_dome()], WINDOW),
    ]
    if p.mount_holes:
        parts.append(("base_screws", make_corner_screws(), SCREW))
    if p.connector:
        conn = make_connector()
        parts += [("connector", conn["connector"], METAL),
                  ("connector_insert", conn["connector_insert"], INSERT_BLACK),
                  ("connector_pins", conn["pins"], GOLD),
                  ("flange_screws", conn["screws"], SCREW)]
    if p.fov_range > 0:
        parts.append(("fov", [make_fov(p)], FOV_BLUE))

    loc = cq.Location((0, 0, -OPTICAL_Z if p.origin == "optical" else 0.0))
    assy = cq.Assembly(name="mid360s")
    for name, solids, color in parts:
        shape = solids[0] if len(solids) == 1 else cq.Compound.makeCompound(solids)
        assy.add(shape, name=name, loc=loc, color=cq.Color(*color))
    return assy


def add_param_args(parser: argparse.ArgumentParser, cls) -> None:
    """One flag per dataclass field. Default None marks flags the user did not give."""
    for f in dataclasses.fields(cls):
        flag = "--" + f.name.replace("_", "-")
        if f.type is bool:
            parser.add_argument(flag, dest=f.name, action=argparse.BooleanOptionalAction,
                                default=None, help=f"default: {f.default}")
        else:
            default = f"{f.default:.4g}" if f.type is float else f.default
            parser.add_argument(flag, dest=f.name, type=f.type, default=None,
                                choices=f.metadata.get("choices"), help=f"default: {default}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a Livox Mid-360S LiDAR STEP model.")
    add_param_args(parser, Mid360Params)
    parser.add_argument("-o", "--output", default="mid360s.step", help="STEP output path")
    args = vars(parser.parse_args())
    output = args.pop("output")
    try:
        p = Mid360Params(**{k: v for k, v in args.items() if v is not None})
    except ValueError as e:
        parser.error(str(e))

    assy = build_mid360s(p)
    assy.export(output)
    names = [c.name for c in assy.children if c.name != "fov"]
    bb = cq.Compound.makeCompound(
        [c.obj.moved(c.loc) for c in assy.children if c.name in names]).BoundingBox()
    oz = 0.0 if p.origin == "optical" else OPTICAL_Z
    print(f"Wrote {output}")
    print(f"Parts: {', '.join(c.name for c in assy.children)}")
    print(f"Sensor size: X {bb.xlen:.2f}  Y {bb.ylen:.2f}  Z {bb.zlen:.2f} mm "
          f"(X {bb.xmin:.2f} to {bb.xmax:.2f}, Z {bb.zmin:.2f} to {bb.zmax:.2f})")
    print(f"Optical center: (0, 0, {oz:.2f})")


if __name__ == "__main__":
    main()
