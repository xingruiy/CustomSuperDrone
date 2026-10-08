"""3D printed battery holder for the SUPER quadrotor, as STEP and STL files.

A parametric copy of the SUPER-Hardware print (hardware/3DPrinting/battery_board.stl).
With the default values it matches that STL. The flange screws to the underside
of the battery plate, with the four M3 screws that go up into the lower pillars.
The battery sits in the cradle below it, held by a strap through the two
bridges. A carbon foot screws to each end block.

Parts of the holder:
    Flange: a rounded plate with a rounded top edge. The cradle opening goes
    through it.
    Cradle: two end blocks and two side walls, with fillets to the flange. The
    +Y wall has a gap in the middle.
    Ribs: four sloped ribs on the outside of the side walls, in line with the bridges.
    Strap bridges: two cross beams at the top of the cradle, between the side
    walls, each with two holes.
    Holes: four flange screw holes (battery plate and pillars), one vertical hole
    through each end block, and two foot screw holes in each end face. Each foot
    screw hole ends in a hex nut trap that opens into the cradle.

Coordinates, all in mm: X along the long side, Y across, Z up. Z = 0 is the
flange face (against the battery plate), and the cradle points to +Z. The
origin is the holder center. In the drone (super_drone.py) the holder is turned
over, and its X runs along the drone Y.

The STL is in the same orientation as the STEP file.

Usage:
    uv run python battery_holder.py
    uv run python battery_holder.py --height 20 --inner-w 30 -o battery_holder_wide.step --stl battery_holder_wide.stl
"""

import argparse
import math
from dataclasses import dataclass

import cadquery as cq

from nuc_shield import PETG_DENSITY, add_param_args


@dataclass
class HolderParams:
    # Flange
    length: float = 95.0  # along X; the end faces carry the feet
    width: float = 56.0
    flange_t: float = 4.0
    flange_r: float = 5.0  # corner radius
    flange_edge_r: float = 2.0  # round on the outer top edge
    # Cradle
    height: float = 17.0  # flange face to the bottom of the cradle
    inner_w: float = 26.0  # between the side walls
    wall_t: float = 3.0
    end_block_l: float = 9.5
    corner_r: float = 2.0  # vertical corners: inside the cradle and outside the end blocks
    wall_gap: float = 30.0  # gap in the middle of the +Y side wall
    gap_r: float = 4.0  # round bottom corners of the gap
    fillet_r: float = 3.0  # walls and ribs to the flange, bridges to the walls
    # Ribs, at x = +-bridge_x, from the top of the wall down to the flange edge
    rib_t: float = 4.0
    # Strap bridges
    bridge_x: float = 20.0  # bridge centers at x = +-bridge_x
    bridge_l: float = 10.0  # along X
    bridge_t: float = 6.0  # along Z, from the bottom of the cradle
    bridge_r: float = 10.0  # fillet from the inner face of each bridge to the full side wall
    bridge_hole_d: float = 3.3
    bridge_hole_y: float = 5.5  # two holes in each bridge, at y = +-bridge_hole_y
    # Holes
    plate_hole_d: float = 3.2  # M3, to the lower pillars
    plate_hole_x: float = 40.5
    plate_hole_y: float = 22.5
    end_hole_d: float = 3.3  # vertical, through each end block
    end_hole_x: float = 42.75
    foot_hole_d: float = 3.1  # M3, along X into the end faces
    foot_hole_y: float = 8.5  # holes at y = +-foot_hole_y
    foot_hole_z: float = 5.0  # from the flange face
    nut_af: float = 5.6  # M3 nut trap, across flats (corners up and down)
    nut_depth: float = 4.5  # from the inner face of the end block

    @property
    def cradle_w(self) -> float:
        return self.inner_w + 2 * self.wall_t

    @property
    def nut_ac(self) -> float:
        return self.nut_af * 2 / math.sqrt(3)

    def __post_init__(self):
        for name, value in vars(self).items():
            if value <= 0:
                raise ValueError(f"{name} must be positive, got {value}")
        hx, wo = self.length / 2, self.cradle_w / 2
        b0, b1 = self.bridge_x - self.bridge_l / 2, self.bridge_x + self.bridge_l / 2
        if wo + self.fillet_r > self.width / 2 - self.flange_edge_r:
            raise ValueError("the cradle and its fillets are wider than the flange: reduce inner_w or wall_t")
        if self.flange_edge_r >= self.flange_t or self.flange_r * 2 > min(self.length, self.width):
            raise ValueError("flange_edge_r or flange_r is too large for the flange")
        if self.corner_r * 2 >= min(self.inner_w, self.end_block_l):
            raise ValueError("corner_r is too large")
        if self.wall_gap / 2 > b0:
            raise ValueError("wall_gap must end at or before the inner faces of the bridges")
        if self.gap_r > min(self.wall_gap / 2, self.height - self.flange_t):
            raise ValueError("gap_r is too large for the gap")
        if self.bridge_t + self.fillet_r >= self.height - self.flange_t:
            raise ValueError("bridge_t must leave space between the bridges and the flange")
        if b1 + self.fillet_r > hx - self.end_block_l - self.corner_r:
            raise ValueError("the bridges must stay clear of the end blocks")
        if self.bridge_r > min(b0, self.inner_w - self.fillet_r):
            raise ValueError("bridge_r is too large")
        if self.rib_t / 2 + self.fillet_r > self.bridge_x - self.wall_gap / 2 + 1e-9:
            raise ValueError("the ribs and their fillets must stay clear of the wall gap")
        if self.bridge_hole_y + self.bridge_hole_d / 2 > self.inner_w / 2 - self.fillet_r:
            raise ValueError("bridge_hole_y puts the bridge holes into the fillets")
        if not wo + self.fillet_r < self.plate_hole_y < self.width / 2 - self.plate_hole_d / 2:
            raise ValueError("plate_hole_y must put the flange screw holes outside the cradle, on the flange")
        if self.end_hole_x + self.end_hole_d / 2 > hx or self.end_hole_x - self.end_hole_d / 2 < hx - self.end_block_l:
            raise ValueError("end_hole_x must put the vertical holes inside the end blocks")
        if self.foot_hole_y + self.nut_af / 2 > self.inner_w / 2:
            raise ValueError("the nut traps must open into the cradle")
        if not self.nut_ac / 2 < self.foot_hole_z < self.height - self.nut_ac / 2:
            raise ValueError("foot_hole_z puts the nut traps outside the end blocks")
        if self.nut_depth >= self.end_block_l:
            raise ValueError("nut_depth must be less than end_block_l")
        if self.foot_hole_y - self.nut_af / 2 <= self.end_hole_d / 2:
            raise ValueError("the nut traps cut into the vertical end holes: increase foot_hole_y")


def box(x0, x1, y0, y1, z0, z1) -> cq.Solid:
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def near(a: float, b: float) -> bool:
    return abs(a - b) < 1e-4


def corner_fillet(cx: float, cy: float, sx: int, sy: int, r: float, z0: float, z1: float) -> cq.Shape:
    """Concave fillet in plan view: the corner block between (cx, cy) and (cx + sx r, cy + sy r),
    minus a cylinder at its far corner. Extruded from z0 to z1."""
    x0, x1 = sorted((cx, cx + sx * r))
    y0, y1 = sorted((cy, cy + sy * r))
    tool = cq.Solid.makeCylinder(r, z1 - z0 + 2, cq.Vector(cx + sx * r, cy + sy * r, z0 - 1))
    return box(x0, x1, y0, y1, z0, z1).cut(tool)


def mirrored(shape: cq.Shape, planes: str) -> list[cq.Shape]:
    """The shape and its mirror images in the YZ plane ("x"), the XZ plane ("y") or both ("xy")."""
    out = [shape]
    for axis, plane in (("x", "YZ"), ("y", "XZ")):
        if axis in planes:
            out += [s.mirror(plane) for s in out]
    return out


def build_ribs(p: HolderParams) -> tuple[list[cq.Shape], list[cq.Shape]]:
    """The four ribs, and the fillets between the ribs and the flange.

    A rib slopes from the top of the wall down to the flange edge. Its fillets to
    the flange are built here, not with a CAD fillet, because the rib face shrinks
    to nothing at the flange edge. They are cut by the rib top plane, and reach down
    to the bottom of the flange edge round, so the edge is square next to the rib.
    Where a rib, the wall and the flange meet, a spherical corner joins the fillets."""
    wo, hy, r = p.cradle_w / 2, p.width / 2, p.fillet_r
    x0, x1 = p.bridge_x - p.rib_t / 2, p.bridge_x + p.rib_t / 2
    slope = [(wo - 1, 0), (hy, 0), (hy, p.flange_t), (wo, p.height), (wo - 1, p.height)]
    rib = cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(slope).close().extrude(p.rib_t).val()
    below_top = cq.Workplane("YZ", origin=(x0 - r - 1, 0, 0)).polyline(slope).close().extrude(p.rib_t + 2 * r + 2).val()
    z0, z1 = p.flange_t - p.flange_edge_r, p.flange_t + r
    pieces = []
    for fx, dx in ((x0, -1), (x1, 1)):
        xa, xb = sorted((fx, fx + dx * r))
        tool = cq.Solid.makeCylinder(r, hy - wo + 4, cq.Vector(fx + dx * r, wo - 2, z1), cq.Vector(0, 1, 0))
        pieces.append(box(xa, xb, wo - 1, hy, z0, z1).cut(tool))
        ball = cq.Solid.makeSphere(r, cq.Vector(fx + dx * r, wo + r, z1), angleDegrees1=-90, angleDegrees2=90)
        pieces.append(box(xa, xb, wo, wo + r, p.flange_t, z1).cut(ball))
    fillets = cq.Compound.makeCompound(pieces).intersect(below_top)
    return mirrored(rib, "xy"), mirrored(fillets, "xy")


def build_bridge(p: HolderParams) -> cq.Shape:
    """The +X strap bridge, with its fillets to the walls in plan view and underneath."""
    wi, r = p.inner_w / 2, p.fillet_r
    b0, b1 = p.bridge_x - p.bridge_l / 2, p.bridge_x + p.bridge_l / 2
    z0, z1 = p.height - p.bridge_t, p.height
    # Plan view outline, from z0 - r so the underside fillets can be cut from it
    outline = [box(b0, b1, -wi, wi, z0 - r, z1),
               corner_fillet(b1, -wi, 1, 1, r, z0 - r, z1),  # outer face, both walls
               corner_fillet(b1, wi, 1, -1, r, z0 - r, z1),
               corner_fillet(b0, -wi, -1, 1, p.bridge_r, z0 - r, z1)]  # inner face, full wall only
    outline = outline[0].fuse(*outline[1:]).clean()
    # Underside: the bridge itself plus a fillet along each wall
    xa, xb = b0 - p.bridge_r - 1, b1 + r + 1
    keep = [box(xa, xb, -wi, wi, z0, z1)]
    for sy in (-1, 1):
        y0, y1 = sorted((sy * wi, sy * (wi - r)))
        tool = cq.Solid.makeCylinder(r, xb - xa + 2, cq.Vector(xa - 1, sy * (wi - r), z0 - r), cq.Vector(1, 0, 0))
        keep.append(box(xa, xb, y0, y1, z0 - r, z0).cut(tool))
    bridge = outline.intersect(keep[0].fuse(*keep[1:]))
    holes = [cq.Solid.makeCylinder(p.bridge_hole_d / 2, p.bridge_t + 2, cq.Vector(p.bridge_x, y, z0 - 1))
             for y in (-p.bridge_hole_y, p.bridge_hole_y)]
    return bridge.cut(*holes)


def nut_trap(p: HolderParams, y: float) -> cq.Shape:
    """+X hex nut trap: from just inside the cradle to nut_depth into the end block."""
    x0 = p.length / 2 - p.end_block_l - 0.1
    rc = p.nut_ac / 2
    pts = [(y + rc * math.cos(math.radians(a)), p.foot_hole_z + rc * math.sin(math.radians(a)))
           for a in range(30, 360, 60)]
    return cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(p.nut_depth + 0.1).val()


def build_holder(p: HolderParams) -> cq.Shape:
    hx, hy = p.length / 2, p.width / 2
    wo, wi, px = p.cradle_w / 2, p.inner_w / 2, p.length / 2 - p.end_block_l
    flange = (cq.Workplane("XY").rect(p.length, p.width).extrude(p.flange_t)
              .edges("|Z").fillet(p.flange_r).faces(">Z").edges().fillet(p.flange_edge_r).val())
    cradle = cq.Workplane("XY").rect(p.length, p.cradle_w).extrude(p.height).edges("|Z").fillet(p.corner_r).val()
    ribs, rib_fillets = build_ribs(p)
    holder = flange.fuse(cradle, *ribs).clean()

    # Fillets: walls to the flange, ribs to the walls
    rib_faces = (p.bridge_x - p.rib_t / 2, p.bridge_x + p.rib_t / 2)
    def concave(e: cq.Edge) -> bool:
        c, bb = e.Center(), e.BoundingBox()
        on_wall = near(abs(c.y), wo)
        wall_base = on_wall and near(bb.zmin, p.flange_t) and near(bb.zmax, p.flange_t) and bb.xlen > p.corner_r
        rib_wall = on_wall and any(near(abs(c.x), f) for f in rib_faces) and bb.zlen > p.fillet_r
        return wall_base or rib_wall
    holder = holder.fillet(p.fillet_r, [e for e in holder.Edges() if concave(e)])
    holder = holder.fuse(*rib_fillets)

    # Cradle opening (through the flange) and the gap in the +Y wall
    opening = (cq.Workplane("XY", origin=(0, 0, -1)).rect(2 * px, p.inner_w).extrude(p.height + 2)
               .edges("|Z").fillet(p.corner_r).val())
    gap = (cq.Workplane("XZ", origin=(0, hy + 1, 0)).center(0, p.flange_t + p.height / 2)
           .rect(p.wall_gap, p.height).extrude(hy + 1 - (wi - 1))
           .edges("|Y and <Z").fillet(p.gap_r).val())
    holder = holder.cut(opening, gap)
    holder = holder.fuse(*mirrored(build_bridge(p), "x")).clean()

    def vertical(d, x, y, h):
        return cq.Solid.makeCylinder(d / 2, h + 2, cq.Vector(x, y, -1))

    holes = [vertical(p.plate_hole_d, sx * p.plate_hole_x, sy * p.plate_hole_y, p.flange_t)
             for sx in (-1, 1) for sy in (-1, 1)]
    holes += [vertical(p.end_hole_d, sx * p.end_hole_x, 0, p.height) for sx in (-1, 1)]
    # Foot screw holes: along X from the end faces to the nut traps.
    for y in (-p.foot_hole_y, p.foot_hole_y):
        hole = cq.Solid.makeCylinder(p.foot_hole_d / 2, p.end_block_l - p.nut_depth + 0.2,
                                     cq.Vector(hx + 0.1, y, p.foot_hole_z), cq.Vector(-1, 0, 0))
        holes += mirrored(hole.fuse(nut_trap(p, y)), "x")
    return holder.cut(*holes).clean()


# ---------------------------------------------------------------- CLI


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the SUPER battery holder (STEP and print-ready STL).")
    add_param_args(parser, HolderParams)
    parser.add_argument("-o", "--output", default="battery_holder.step", help="STEP output path")
    parser.add_argument("--stl", default="battery_holder.stl", help="STL output path")
    args = vars(parser.parse_args())
    output, stl = args.pop("output"), args.pop("stl")
    try:
        p = HolderParams(**{k: v for k, v in args.items() if v is not None})
    except ValueError as e:
        parser.error(str(e))

    holder = build_holder(p)
    cq.exporters.export(holder, output)
    cq.exporters.export(holder, stl, tolerance=0.02, angularTolerance=0.1)
    bb = holder.BoundingBox()
    volume = holder.Volume() / 1000.0
    print(f"Wrote {output} and {stl}")
    print(f"Size: X {bb.xlen:.2f}  Y {bb.ylen:.2f}  Z {bb.zlen:.2f} mm")
    print(f"Volume: {volume:.1f} cm3, about {volume * PETG_DENSITY:.0f} g in PETG at 100% fill")


if __name__ == "__main__":
    main()
