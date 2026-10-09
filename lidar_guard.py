"""3D printed guard for the Livox Mid-360S on the SUPER quadrotor, as STEP and STL files.

The form follows the open-cage housings that people print for the Mid-360 (for
example "Protective housing for Livox Mid-360" by Michal Pelka on Printables):
four thin ribs that arch over the dome and meet in a small cap, on a low band
around the sensor base. If the drone turns over or hits a wall, the ribs take
the hit, not the dome window.

Parts of the guard:
    Band: a low rounded-square ring around the sensor base, on the top plate.
    Feet: one at each corner, over the four M3 holes at (+-35, +-35) on the
    diagonal spokes of the top plate, so the plate needs no new holes.
    Ribs: one on each diagonal, in the vertical plane through the sensor axis.
    Each rib rises from its foot next to the body corner, then follows a quarter
    ellipse over the dome. The rib is deeper at the bottom than at the top.
    Cap: a round boss where the ribs meet above the dome, with a hole in it.

Field of view: the Mid-360S sees 360 degrees around and -7 to +52 degrees up. A
guard that covers the dome must cross this view, so each rib makes a narrow blind
sector. The ribs are thin across the view (rib_w) and stand on the diagonals, away
from the connector. The script reports the blocked part of the FOV (ray cast).
Points that the ribs return are within the blind range of most LiDAR drivers and
SLAM front ends (for example 0.5 m), so they are filtered out. The loss is the
view behind the ribs.

Fixing: one M3 x 10 screw from under the top plate into each foot, into an M3
nut. The nut slides into a trap that opens to the outside of the foot.

Coordinates, all in mm, the same as mid360s.py:
    Z: up. Z = 0 is the top face of the top plate (the sensor's bottom face).
    The origin is the sensor center. +X is the M12 connector side. The ribs are
    on the diagonals.

Printing: the STL is upright, with the band and feet on the bed. The top part
of each arch and the cap need supports (tree supports from the bed). PETG or ASA
is better than PLA for impacts.

Usage:
    uv run python lidar_guard.py
    uv run python lidar_guard.py --rib-w 3 --dome-gap 8 -o lidar_guard_b.step --stl lidar_guard_b.stl
"""

import argparse
import dataclasses
import math
from dataclasses import dataclass

import cadquery as cq
from OCP.gp import gp_Dir, gp_Lin, gp_Pnt
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector

from mid360s import (BASE_W, CORNER_R, FOV_MAX, FOV_MIN, OPTICAL_Z, Mid360Params, add_param_args,
                     build_mid360s)
from mid360s import TOP_Z as DOME_TOP_Z
from nuc_shield import PETG_DENSITY

RIB_ANGLES = (45.0, 135.0, 225.0, 315.0)  # rib directions, degrees from +X
MIN_SENSOR_GAP = 0.5  # smallest space between the guard and the sensor


@dataclass
class GuardParams:
    # Feet, on the four M3 holes of the top plate
    hole_xy: float = 35.0  # hole centers at (+-hole_xy, +-hole_xy)
    hole_d: float = 3.2
    foot_w: float = 9.0  # across the rib; the foot runs the full rib depth along it
    foot_h: float = 8.0  # straight part; above it the foot tapers into the rib
    taper_h: float = 6.0
    nut_af: float = 5.6  # M3 nut trap, across flats; it opens to the outside of the foot
    nut_z: float = 3.5  # bottom of the nut trap, above the plate
    nut_h: float = 2.6
    # Band around the sensor base
    band_gap: float = 0.8  # to the sensor base
    band_w: float = 3.5
    band_h: float = 4.0
    # Ribs
    rib_w: float = 3.5  # across the view direction: sets the width of the blind sectors
    rib_in: float = 4.5  # inner face of the rib, inside the hole center
    rib_depth: float = 10.0  # radial, at the bottom
    rib_top: float = 6.0  # vertical, at the crown
    spring_z: float = 20.0  # where the arch starts; the ribs are vertical below it
    dome_gap: float = 6.0  # arch crowns to the top of the dome window (the cap is 1 mm lower)
    edge_r: float = 1.0  # round on the rib edges
    # Cap
    cap_r: float = 9.0
    cap_hole_d: float = 5.0
    cap_rise: float = 1.5  # cap top above the arch crowns

    @property
    def rib_r(self) -> float:
        """Distance from the sensor axis to the rib (and foot) centers."""
        return self.hole_xy * math.sqrt(2)

    @property
    def nut_ac(self) -> float:
        return self.nut_af * 2 / math.sqrt(3)

    @property
    def arch(self) -> tuple[float, float, float, float]:
        """Inner and outer ellipse semi-axes of the arch: (a_in, b_in, a_out, b_out)."""
        a_in = self.rib_r - self.rib_in
        a_out = a_in + self.rib_depth
        b_in = DOME_TOP_Z + self.dome_gap - self.spring_z
        return a_in, b_in, a_out, b_in + self.rib_top

    def __post_init__(self):
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if value <= 0:
                raise ValueError(f"{f.name} must be positive, got {value}")
        if self.hole_d >= self.nut_af or self.foot_w < self.nut_af + 2.0:
            raise ValueError("foot_w must leave 1 mm of wall beside the nut trap, and hole_d must be under nut_af")
        if self.rib_in < self.nut_ac / 2 + 1.0 or self.rib_depth - self.rib_in < self.nut_ac / 2 + 1.0:
            raise ValueError("the foot must reach 1 mm past the nut trap at both ends: check rib_in and rib_depth")
        if self.nut_z < 1.5 or self.nut_z + self.nut_h + 1.5 > self.foot_h:
            raise ValueError("the nut trap must be at least 1.5 mm above the plate and 1.5 mm under the top of the pad")
        if self.rib_w > self.foot_w or self.rib_depth < self.rib_w:
            raise ValueError("rib_w must not be wider than foot_w or deeper than rib_depth")
        if self.edge_r * 2 >= min(self.rib_w, self.rib_top):
            raise ValueError("edge_r must be under half of rib_w and rib_top")
        if not self.band_h < self.spring_z < DOME_TOP_Z:
            raise ValueError(f"spring_z must be between band_h and the dome top ({DOME_TOP_Z})")
        if self.cap_hole_d >= self.cap_r * 2 - 4.0 or self.cap_r <= self.rib_w:
            raise ValueError("the cap must be wider than the ribs, with 2 mm of wall around its hole")
        if BASE_W / 2 + self.band_gap + self.band_w > self.hole_xy + self.foot_w / 2:
            raise ValueError("the band is outside the feet: reduce band_gap or band_w")


# ---------------------------------------------------------------- parts


def box(x0, x1, y0, y1, z0, z1) -> cq.Solid:
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def build_arch(p: GuardParams) -> cq.Shape:
    """Two opposite ribs as one arch along X: a half elliptic ring on two vertical legs.
    (Two arches cross at the axis. Four ribs that end on the axis do not fuse well.)"""
    a_in, b_in, a_out, b_out = p.arch
    w, zc = p.rib_w, p.spring_z

    def ellipse(a, b):
        return cq.Workplane("XZ").center(0, zc).ellipse(a, b).extrude(w / 2, both=True).val()
    upper = box(-a_out - 1, a_out + 1, -w, w, zc, zc + b_out + 1)
    arch = ellipse(a_out, b_out).intersect(upper).cut(ellipse(a_in, b_in))
    legs = [box(a_in, a_out, -w / 2, w / 2, 0, zc), box(-a_out, -a_in, -w / 2, w / 2, 0, zc)]
    arch = arch.fuse(*legs).clean()
    # Round the edges along the arch, not the ones on the plate
    return arch.fillet(p.edge_r, [e for e in arch.Edges() if e.BoundingBox().zmax > 0.01])


def build_foot(p: GuardParams) -> cq.Shape:
    """A rounded pad under the +X rib, as long as the rib is deep. Above foot_h it tapers
    to the rib width."""
    a_in, _, a_out, _ = p.arch
    wp = cq.Workplane("XY").center((a_in + a_out) / 2, 0)
    pad = wp.slot2D(a_out - a_in, p.foot_w).extrude(p.foot_h)
    taper = (wp.workplane(offset=p.foot_h).slot2D(a_out - a_in, p.foot_w)
             .workplane(offset=p.taper_h).slot2D(a_out - a_in, p.rib_w).loft())
    return pad.union(taper).val()


def foot_tools(p: GuardParams) -> list[cq.Shape]:
    """Screw hole, hex nut trap (flats along X) and the slot from the trap to the outside."""
    hole = cq.Solid.makeCylinder(p.hole_d / 2, p.nut_z + p.nut_h + 3.0, cq.Vector(p.rib_r, 0, -1))
    trap = cq.Workplane("XY", origin=(p.rib_r, 0, p.nut_z)).polygon(6, p.nut_ac).extrude(p.nut_h).val()
    slot = box(p.rib_r, p.rib_r + p.rib_depth, -p.nut_af / 2, p.nut_af / 2, p.nut_z, p.nut_z + p.nut_h)
    return [hole, trap, slot]


def build_band(p: GuardParams) -> cq.Shape:
    half_in = BASE_W / 2 + p.band_gap
    r_in = CORNER_R + p.band_gap

    def square(half, r, z0, h):
        return cq.Workplane("XY", origin=(0, 0, z0)).rect(2 * half, 2 * half).extrude(h).edges("|Z").fillet(r).val()
    outer = square(half_in + p.band_w, r_in + p.band_w, 0, p.band_h)
    return outer.cut(square(half_in, r_in, -1, p.band_h + 2))


def build_cap(p: GuardParams) -> cq.Shape:
    """A round boss over the crossing of the arches. It stands proud of the arch crowns
    (cap_rise on top, 1 mm under), so its flat faces are not tangent to them."""
    _, b_in, _, b_out = p.arch
    z0, z1 = p.spring_z + b_in - 1.0, p.spring_z + b_out + p.cap_rise
    cap = (cq.Workplane("XY", origin=(0, 0, z0)).circle(p.cap_r).extrude(z1 - z0)
           .faces(">Z").edges().fillet(1.5).faces("<Z").edges().chamfer(0.8))
    return cap.faces(">Z").workplane().hole(p.cap_hole_d).val()


def rotated(shape: cq.Shape, angle: float) -> cq.Shape:
    return shape.rotate(cq.Vector(), cq.Vector(0, 0, 1), angle)


def build_guard(p: GuardParams) -> cq.Shape:
    arches = [rotated(build_arch(p), a) for a in RIB_ANGLES[:2]]
    feet = [rotated(build_foot(p), a) for a in RIB_ANGLES]
    guard = build_band(p).fuse(build_cap(p), *arches, *feet).clean()
    tools = [rotated(t, a) for a in RIB_ANGLES for t in foot_tools(p)]
    return guard.cut(*tools).clean()


# ---------------------------------------------------------------- checks


def check_guard(guard: cq.Shape) -> float:
    """Raise ValueError if the guard is too near the sensor. Return the space to the sensor."""
    sensor = build_mid360s(Mid360Params(groove_count=0, knurl_count=0, mount_holes=False)).toCompound()
    gap = guard.distance(sensor)
    if gap < MIN_SENSOR_GAP:
        raise ValueError(f"the guard is {gap:.2f} mm from the sensor (at least {MIN_SENSOR_GAP} needed)")
    return gap


def blocked_fraction(guard: cq.Shape, step: float = 0.25) -> float:
    """Ray cast from the optical center over the FOV. Return the part of the FOV (by solid
    angle) that the guard blocks. The guard has 4-fold symmetry, so 90 degrees is enough."""
    inter = IntCurvesFace_ShapeIntersector()
    inter.Load(guard.wrapped, 1e-6)
    origin = gp_Pnt(0, 0, OPTICAL_Z)
    hit = total = 0.0
    for i in range(int(FOV_MAX - FOV_MIN)):
        el = math.radians(FOV_MIN + i + 0.5)
        weight = math.cos(el)
        for j in range(int(90 / step)):
            az = math.radians((j + 0.5) * step)
            ray = gp_Dir(math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
            inter.Perform(gp_Lin(origin, ray), 0.0, 1000.0)
            total += weight
            if inter.NbPnt() > 0:
                hit += weight
    return hit / total


# ---------------------------------------------------------------- CLI


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the Mid-360S guard (STEP and print-ready STL).")
    add_param_args(parser, GuardParams)
    parser.add_argument("-o", "--output", default="lidar_guard.step", help="STEP output path (sensor frame)")
    parser.add_argument("--stl", default="lidar_guard.stl", help="STL output path (upright, as printed)")
    args = vars(parser.parse_args())
    output, stl = args.pop("output"), args.pop("stl")
    try:
        p = GuardParams(**{k: v for k, v in args.items() if v is not None})
        guard = build_guard(p)
        gap = check_guard(guard)
    except ValueError as e:
        parser.error(str(e))

    cq.exporters.export(guard, output)
    cq.exporters.export(guard, stl, tolerance=0.02, angularTolerance=0.1)
    bb = guard.BoundingBox()
    volume = guard.Volume() / 1000.0
    print(f"Wrote {output} and {stl}")
    print(f"Size: X {bb.xlen:.2f}  Y {bb.ylen:.2f}  Z {bb.zlen:.2f} mm ({bb.zmax - DOME_TOP_Z:.1f} mm above the dome)")
    print(f"Volume: {volume:.1f} cm3, about {volume * PETG_DENSITY:.0f} g in PETG at 100% fill")
    print(f"Space to the sensor: {gap:.2f} mm")
    print(f"FOV blocked: {100 * blocked_fraction(guard):.1f} % (ray cast)")
    print("Hardware: 4 x M3 x 10 screws from under the top plate, 4 x M3 nuts")


if __name__ == "__main__":
    main()
