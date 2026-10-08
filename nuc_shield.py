"""3D printed NUC shield for the SUPER quadrotor, as STEP and STL files.

The shield is one closed ring around the NUC 13 Pro board, between the main
plate and the top plate. A boss at each corner has a vertical bore. The bores
slide down over the four 40 mm aluminum pillars, so the pillars hold the ring
in X and Y, and the two plates hold it in Z. To fit it, remove the top plate
(4 screws; the LiDAR stays on it), lower the ring over the pillars, and put
the top plate back.

Openings:
    Front and back walls: a port opening, open at the bottom edge. It clears the
    plugs for all back panel ports (HDMI, LAN, USB, Thunderbolt, DC in) and the
    front USB-A, audio jack and power button. Vent slots above the front opening
    let intake air in. The back opening is taller, so the cooler fin outlet is
    fully open.
    Side walls: vent slots for the blower intake. Small notches at the top and
    bottom edges keep the five M3 holes (y = +-60.5) in both plates free.
    Corners: a small relief at the bottom outer tip clears the LED bars on the arms.

Coordinates, all in mm, the same as the drone (super_drone.py) but with Z = 0
at the bottom face of the shield (the top face of the main plate):
    X forward (NUC front panel), Y left, Z up. The origin is the frame center.

Printing: the STL is turned over, so the top edge of the ring is on the bed.
The port openings are then open at the top of the print, and the ring needs no
supports. PETG or ASA is better than PLA near the NUC exhaust.

Usage:
    uv run python nuc_shield.py
    uv run python nuc_shield.py --bore-d 5.8 --wall-t 2.0 -o nuc_shield_thick.step --stl nuc_shield_thick.stl
"""

import argparse
import dataclasses
import math
from dataclasses import dataclass

import cadquery as cq

SIDE_HOLES_X = (-20.0, -10.0, 0.0, 10.0, 20.0)  # M3 holes at y = +-60.5 in both plates
PETG_DENSITY = 1.27  # g/cm3


@dataclass
class ShieldParams:
    # Frame
    pillar_xy: float = 57.5  # pillar centers at (+-pillar_xy, +-pillar_xy)
    pillar_d: float = 5.0
    plate_gap: float = 40.0  # main plate top to top plate bottom (pillar length)
    top_gap: float = 0.3  # space between the shield and the top plate
    # Ring
    half_size: float = 62.6  # outer face of the walls; the top plate edge is at 62.5
    wall_t: float = 1.6
    bore_d: float = 5.6  # pillar bore, with clearance for print and pillar position errors
    bore_chamfer: float = 0.6  # lead-in at both ends of each bore
    # Port openings in the front and back walls, open at the bottom edge
    port_w: float = 92.0
    port_h: float = 20.0  # front: USB-A, audio jack, power button
    back_port_h: float = 31.0  # back: all ports and the cooler fin outlet (top at z = 29.6)
    # Vent slots (vertical, round ends)
    vent_w: float = 3.5
    vent_pitch: float = 7.0
    front_vent_span: float = 86.0  # front wall, along Y
    front_vent_z: tuple = (23.0, 35.0)
    side_vent_span: float = 90.0  # side walls, along X
    side_vent_z: tuple = (9.0, 33.0)
    # Notches that keep the plate M3 holes free (screw heads, nuts)
    hole_notches: bool = True
    notch_w: float = 7.0
    notch_d: float = 2.5
    # Relief at the bottom outer tip of each corner, for the LED bars on the arms
    corner_relief_r: float = 85.3  # distance from the frame center, along the diagonal
    corner_relief_h: float = 4.5

    @property
    def height(self) -> float:
        return self.plate_gap - self.top_gap

    @property
    def boss_r(self) -> float:
        return self.half_size - self.pillar_xy

    def __post_init__(self):
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if f.type is float and value <= 0 and f.name not in ("top_gap", "bore_chamfer"):
                raise ValueError(f"{f.name} must be positive, got {value}")
        if self.bore_d <= self.pillar_d:
            raise ValueError("bore_d must be larger than pillar_d")
        if self.boss_r - self.bore_d / 2 < 1.2:
            raise ValueError("the boss wall around the bore is under 1.2 mm: increase half_size or reduce bore_d")
        if self.half_size - self.wall_t < self.pillar_xy + self.pillar_d / 2:
            raise ValueError("the walls must be outside the pillars: increase half_size or reduce wall_t")
        if self.port_w / 2 > self.pillar_xy - self.boss_r - 2.0:
            raise ValueError("port_w is too wide: the port opening cuts into the corner bosses")
        for name in ("port_h", "back_port_h"):
            if getattr(self, name) > self.height - 8.0:
                raise ValueError(f"{name} must leave at least 8 mm of wall above it")
        for name, top in (("front_vent_z", self.height), ("side_vent_z", self.height)):
            z0, z1 = getattr(self, name)
            if not 0 < z0 < z1 < top - 3.0 or z1 - z0 < self.vent_w:
                raise ValueError(f"{name} must be inside the wall and longer than vent_w")
        if self.front_vent_z[0] < self.port_h + 2.0:
            raise ValueError("front_vent_z must start at least 2 mm above the port opening")
        if self.front_vent_span / 2 + self.vent_w / 2 > self.pillar_xy - self.boss_r - 1.0:
            raise ValueError("front_vent_span is too wide: the slots cut into the corner bosses")
        if self.side_vent_span / 2 + self.vent_w / 2 > self.pillar_xy - self.boss_r - 1.0:
            raise ValueError("side_vent_span is too wide: the slots cut into the corner bosses")
        if self.vent_pitch <= self.vent_w + 1.0:
            raise ValueError("vent_pitch must leave at least 1 mm between slots")
        if self.hole_notches and self.notch_d >= self.side_vent_z[0] - 2.0:
            raise ValueError("notch_d is too deep for side_vent_z")
        if self.corner_relief_r < self.pillar_xy * math.sqrt(2) + self.bore_d / 2 + 1.0:
            raise ValueError("corner_relief_r cuts into the pillar bore")


# ---------------------------------------------------------------- helpers


def rounded_square(half: float, r: float, h: float) -> cq.Solid:
    return cq.Workplane("XY").rect(2 * half, 2 * half).extrude(h).edges("|Z").fillet(r).val()


def slot_positions(span: float, pitch: float) -> list[float]:
    n = int(math.floor(span / pitch + 1e-9)) + 1
    return [(i - (n - 1) / 2) * pitch for i in range(n)]


def vent_slots(p: ShieldParams, plane: str, span: float, z: tuple) -> cq.Shape:
    """Vertical slots through the front wall (plane "YZ") or through both side walls (plane "XZ")."""
    z0, z1 = z
    reach = p.half_size + 1.0
    # YZ: the normal is +X, so start inside the ring. XZ: the normal is -Y, so start at +Y.
    origin = (p.half_size - p.wall_t - 1.0, 0, 0) if plane == "YZ" else (0, reach, 0)
    length = p.wall_t + 2.0 if plane == "YZ" else 2 * reach
    wp = cq.Workplane(plane, origin=origin)
    pts = [(c, (z0 + z1) / 2) for c in slot_positions(span, p.vent_pitch)]
    return wp.pushPoints(pts).slot2D(z1 - z0, p.vent_w, 90).extrude(length).val()


# ---------------------------------------------------------------- shield


def build_shield(p: ShieldParams) -> cq.Shape:
    h, s, r = p.height, p.half_size, p.boss_r
    ring = rounded_square(s, r, h).cut(rounded_square(s - p.wall_t, max(r - p.wall_t, 0.5), h + 2)
                                       .translate((0, 0, -1)))
    corners = [(x, y) for x in (-p.pillar_xy, p.pillar_xy) for y in (-p.pillar_xy, p.pillar_xy)]
    bosses = [cq.Solid.makeCylinder(r, h, cq.Vector(x, y, 0)) for x, y in corners]
    shield = ring.fuse(*bosses).clean()

    tools = []
    # Pillar bores with a lead-in chamfer at both ends
    for x, y in corners:
        tools.append(cq.Solid.makeCylinder(p.bore_d / 2, h + 2, cq.Vector(x, y, -1)))
        if p.bore_chamfer > 0:
            c = p.bore_chamfer
            for z, d in ((0.0, 1), (h, -1)):
                tools.append(cq.Solid.makeCone(p.bore_d / 2 + c + 1, p.bore_d / 2, c + 1,
                                               cq.Vector(x, y, z - d * 1), cq.Vector(0, 0, d)))
    # Port openings, front and back, open at the bottom edge. The top corners are
    # round, so they do not start cracks.
    for sx, ph in ((1, p.port_h), (-1, p.back_port_h)):
        port = (cq.Workplane("XY", origin=(sx * (s - p.wall_t / 2), 0, -1)).rect(p.wall_t + 4, p.port_w)
                .extrude(ph + 1).edges("|X and >Z").fillet(3.0).val())
        tools.append(port)
    shield = shield.cut(*tools).clean()
    # Vent slots
    shield = shield.cut(vent_slots(p, "YZ", p.front_vent_span, p.front_vent_z),
                        vent_slots(p, "XZ", p.side_vent_span, p.side_vent_z)).clean()
    # Notches over the plate M3 holes, top and bottom edges of the side walls
    if p.hole_notches:
        notches = []
        for x in SIDE_HOLES_X:
            for z0 in (-1.0, h - p.notch_d):
                notches.append(cq.Solid.makeBox(p.notch_w, 2 * s + 2, p.notch_d + 1,
                                                cq.Vector(x - p.notch_w / 2, -s - 1, z0)))
        shield = shield.cut(*notches).clean()
    # Corner relief: cut the bottom of each corner beyond corner_relief_r (along the diagonal)
    relief = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            angle = math.degrees(math.atan2(sy, sx))
            block = cq.Solid.makeBox(20, 40, p.corner_relief_h + 1, cq.Vector(p.corner_relief_r, -20, -1))
            relief.append(block.rotate(cq.Vector(), cq.Vector(0, 0, 1), angle))
    return shield.cut(*relief).clean()


def print_orientation(shield: cq.Shape, p: ShieldParams) -> cq.Shape:
    """Turn the ring over (top edge on the bed) and put its lowest point at z = 0."""
    return shield.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180).translate((0, 0, p.height))


# ---------------------------------------------------------------- CLI


def add_param_args(parser: argparse.ArgumentParser, cls) -> None:
    for f in dataclasses.fields(cls):
        flag = "--" + f.name.replace("_", "-")
        if f.type is bool:
            parser.add_argument(flag, dest=f.name, action=argparse.BooleanOptionalAction,
                                default=None, help=f"default: {f.default}")
        elif f.type is tuple:
            parser.add_argument(flag, dest=f.name, type=float, nargs=2, default=None,
                                metavar=("Z0", "Z1"), help=f"default: {f.default[0]:g} {f.default[1]:g}")
        else:
            parser.add_argument(flag, dest=f.name, type=f.type, default=None,
                                help=f"default: {f.default:.4g}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the SUPER NUC shield (STEP and print-ready STL).")
    add_param_args(parser, ShieldParams)
    parser.add_argument("-o", "--output", default="nuc_shield.step", help="STEP output path (drone frame)")
    parser.add_argument("--stl", default="nuc_shield.stl", help="STL output path (print orientation)")
    args = vars(parser.parse_args())
    output, stl = args.pop("output"), args.pop("stl")
    values = {k: tuple(v) if isinstance(v, list) else v for k, v in args.items() if v is not None}
    try:
        p = ShieldParams(**values)
    except ValueError as e:
        parser.error(str(e))

    shield = build_shield(p)
    cq.exporters.export(shield, output)
    cq.exporters.export(print_orientation(shield, p), stl, tolerance=0.02, angularTolerance=0.1)
    bb = shield.BoundingBox()
    volume = shield.Volume() / 1000.0
    print(f"Wrote {output} and {stl}")
    print(f"Size: X {bb.xlen:.2f}  Y {bb.ylen:.2f}  Z {bb.zlen:.2f} mm")
    print(f"Volume: {volume:.1f} cm3, about {volume * PETG_DENSITY:.0f} g in PETG at 100% fill")


if __name__ == "__main__":
    main()
