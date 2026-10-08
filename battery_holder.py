"""3D printed battery holder for the SUPER quadrotor, as STEP and STL files.

Rebuilt from sections of the SUPER-Hardware print (hardware/3DPrinting/battery_board.stl).
The flange screws to the underside of the battery plate, with the four M3 screws
that go up into the lower pillars. The battery sits in the cradle below it, held
by a strap through the two bridges. A carbon foot screws to each end block.

Parts of the holder:
    Flange: a rounded plate with a window in the middle.
    Cradle: two end blocks and two side walls. The +Y wall has a gap in the middle.
    Strap bridges: two cross beams at the top of the cradle, between the side walls.
    Holes: four flange screw holes (battery plate and pillars), one vertical hole
    through each end block, and two foot screw holes in each end face.

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
    window_l: float = 76.0
    window_w: float = 26.0
    window_r: float = 2.0
    # Cradle
    height: float = 17.0  # flange face to the bottom of the cradle
    inner_w: float = 26.0  # between the side walls
    wall_t: float = 3.0
    end_block_l: float = 9.5
    wall_gap: float = 27.0  # gap in the middle of the +Y side wall
    # Strap bridges
    bridge_x: float = 20.0  # bridge centers at x = +-bridge_x
    bridge_l: float = 10.0  # along X
    bridge_t: float = 6.0  # along Z, from the bottom of the cradle
    # Holes
    plate_hole_d: float = 3.2  # M3, to the lower pillars
    plate_hole_x: float = 40.5
    plate_hole_y: float = 22.5
    end_hole_d: float = 3.3  # vertical, through each end block
    end_hole_x: float = 42.75
    foot_hole_d: float = 3.2  # M3, along X into the end faces
    foot_hole_y: float = 8.5  # holes at y = +-foot_hole_y
    foot_hole_z: float = 5.0  # from the flange face
    foot_hole_depth: float = 9.4

    @property
    def cradle_w(self) -> float:
        return self.inner_w + 2 * self.wall_t

    def __post_init__(self):
        for name, value in vars(self).items():
            if value <= 0:
                raise ValueError(f"{name} must be positive, got {value}")
        if self.cradle_w > self.width:
            raise ValueError("the cradle is wider than the flange: reduce inner_w or wall_t")
        if self.window_l > self.length - 2 * self.end_block_l or self.window_w > self.inner_w:
            raise ValueError("the window must fit inside the cradle")
        if self.flange_r * 2 > min(self.length, self.width) or self.window_r * 2 > min(self.window_l, self.window_w):
            raise ValueError("a corner radius is too large for its outline")
        if self.wall_gap > self.length - 2 * self.end_block_l:
            raise ValueError("wall_gap cuts into the end blocks")
        if self.bridge_t >= self.height - self.flange_t:
            raise ValueError("bridge_t must leave space between the bridges and the flange")
        if not self.wall_gap / 2 <= self.bridge_x - self.bridge_l / 2 < self.bridge_x + self.bridge_l / 2 <= self.length / 2 - self.end_block_l:
            raise ValueError("the bridges must sit between the wall gap and the end blocks")
        if not self.cradle_w / 2 < self.plate_hole_y < self.width / 2 - self.plate_hole_d / 2:
            raise ValueError("plate_hole_y must put the flange screw holes outside the cradle, on the flange")
        if abs(self.end_hole_x) + self.end_hole_d / 2 > self.length / 2 or self.end_hole_x - self.end_hole_d / 2 < self.length / 2 - self.end_block_l:
            raise ValueError("end_hole_x must put the vertical holes inside the end blocks")
        if self.foot_hole_y + self.foot_hole_d / 2 > self.cradle_w / 2 or self.foot_hole_z + self.foot_hole_d / 2 > self.height:
            raise ValueError("the foot screw holes must be inside the end faces")


def box(x0, x1, y0, y1, z0, z1) -> cq.Solid:
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def build_holder(p: HolderParams) -> cq.Shape:
    hx, wo, wi = p.length / 2, p.cradle_w / 2, p.inner_w / 2
    flange = (cq.Workplane("XY").rect(p.length, p.width).extrude(p.flange_t)
              .edges("|Z").fillet(p.flange_r).val())
    window = (cq.Workplane("XY", origin=(0, 0, -1)).rect(p.window_l, p.window_w).extrude(p.flange_t + 2)
              .edges("|Z").fillet(p.window_r).val())
    b0, b1 = p.bridge_x - p.bridge_l / 2, p.bridge_x + p.bridge_l / 2
    body = [
        box(-hx, -hx + p.end_block_l, -wo, wo, 0, p.height),  # end blocks
        box(hx - p.end_block_l, hx, -wo, wo, 0, p.height),
        box(-hx, hx, -wo, -wi, 0, p.height),  # full side wall
        box(-hx, -p.wall_gap / 2, wi, wo, 0, p.height),  # side wall with a gap
        box(p.wall_gap / 2, hx, wi, wo, 0, p.height),
        box(-b1, -b0, -wi, wi, p.height - p.bridge_t, p.height),  # strap bridges
        box(b0, b1, -wi, wi, p.height - p.bridge_t, p.height),
    ]
    holder = flange.cut(window).fuse(*body).clean()

    def vertical(d, x, y, h):
        return cq.Solid.makeCylinder(d / 2, h + 2, cq.Vector(x, y, -1))

    holes = [vertical(p.plate_hole_d, sx * p.plate_hole_x, sy * p.plate_hole_y, p.flange_t)
             for sx in (-1, 1) for sy in (-1, 1)]
    holes += [vertical(p.end_hole_d, sx * p.end_hole_x, 0, p.height) for sx in (-1, 1)]
    # Foot screw holes: along X into the end blocks, starting 0.1 mm outside the end face.
    for sx in (-1, 1):
        for y in (-p.foot_hole_y, p.foot_hole_y):
            holes.append(cq.Solid.makeCylinder(p.foot_hole_d / 2, p.foot_hole_depth + 0.1,
                                               cq.Vector(sx * (hx + 0.1), y, p.foot_hole_z), cq.Vector(-sx, 0, 0)))
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
