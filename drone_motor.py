"""Parametric model of a brushless outrunner drone motor, as a colored STEP assembly.

Two levels of detail (--detail), with the same outer shape:
    full: base with stator tube, stator with teeth, windings, bell (rotor),
    magnets, shaft, two bearings and a circlip.
    simple: outer shape only: base, bell and shaft. The base fills the inside
    of the bell. Faster to build and lighter in CAD.

The Z axis is the motor axis. Z = 0 is the base bottom face (the mounting face).
All units are mm. Default values match a typical 2207 motor (12N14P).

Usage:
    uv run python drone_motor.py
    uv run python drone_motor.py --stator-h 6 --pole-count 12 -o m2206.step
    uv run python drone_motor.py --detail simple -o drone_motor_simple.step
"""

import argparse
import dataclasses
import math
from dataclasses import dataclass, field

import cadquery as cq

Z_AXIS = cq.Vector(0, 0, 1)
ORIGIN = cq.Vector(0, 0, 0)
DETAILS = ("full", "simple")


@dataclass
class MotorParams:
    detail: str = field(default="full", metadata={"choices": DETAILS})  # full: all inner parts, simple: outer shape
    # Stator
    stator_d: float = 22.0  # stator outer diameter
    stator_h: float = 7.0  # stator stack height
    stator_bore: float = 13.0  # stator inner diameter (= stator tube outer diameter)
    slot_count: int = 12  # number of slots (teeth)
    yoke_t: float = 1.5  # radial thickness of the stator yoke ring
    tooth_w: float = 2.2  # tooth width
    tip_t: float = 1.0  # radial thickness of the tooth tips
    slot_open: float = 1.5  # slot opening between tooth tips
    coil_t: float = 0.8  # winding thickness around each tooth
    # Rotor
    pole_count: int = 14  # number of magnets
    magnet_t: float = 1.5  # magnet radial thickness
    magnet_fill: float = 0.85  # magnet arc as a fraction of the pole pitch
    air_gap: float = 0.3  # radial gap between stator and magnets
    bell_wall: float = 1.2  # bell side wall thickness
    bell_top_t: float = 1.5  # bell top thickness
    clearance: float = 1.0  # axial gap between windings and base / bell top
    vent_count: int = 6  # vent holes in bell top (0 = none)
    vent_d: float = 5.0  # vent hole diameter
    vent_pcd: float = 17.0  # vent hole pitch circle diameter
    # Base
    base_d: float = 27.0  # base diameter
    base_h: float = 4.0  # base plate height
    gap: float = 0.5  # axial gap between base and bell
    mount_count: int = 4  # mount holes in base
    mount_pcd: float = 16.0  # mount hole pitch circle diameter
    mount_hole_d: float = 3.0  # mount hole diameter (M3)
    mount_hole_depth: float = 3.5  # mount hole depth
    # Shaft and bearings
    shaft_d: float = 5.0  # shaft diameter
    shaft_len: float = 14.0  # shaft length above bell top
    prop_nut_thread: bool = True  # smaller thread diameter at shaft tip
    bearing_od: float = 11.0  # bearing outer diameter (685: 5 x 11 x 5)
    bearing_w: float = 5.0  # bearing width
    fillet_r: float = 0.8  # edge fillet radius on bell and base

    # Derived sizes

    @property
    def yoke_r(self) -> float:
        return self.stator_bore / 2 + self.yoke_t

    @property
    def tip_in_r(self) -> float:
        return self.stator_d / 2 - self.tip_t

    @property
    def coil_hw(self) -> float:
        """Half width of a winding, measured across the tooth."""
        return self.tooth_w / 2 + self.coil_t

    @property
    def magnet_in_r(self) -> float:
        return self.stator_d / 2 + self.air_gap

    @property
    def magnet_out_r(self) -> float:
        return self.magnet_in_r + self.magnet_t

    @property
    def bell_d(self) -> float:
        return 2 * (self.magnet_out_r + self.bell_wall)

    @property
    def boss_d(self) -> float:
        """Diameter of the shaft boss on the bell top."""
        return self.shaft_d + 4.0

    @property
    def boss_h(self) -> float:
        return 1.5

    # Z positions

    @property
    def stator_z0(self) -> float:
        return self.base_h + self.coil_t + self.clearance

    @property
    def stator_z1(self) -> float:
        return self.stator_z0 + self.stator_h

    @property
    def bell_z0(self) -> float:
        return self.base_h + self.gap

    @property
    def bell_top_z(self) -> float:
        return self.stator_z1 + self.coil_t + self.clearance + self.bell_top_t

    @property
    def magnet_z0(self) -> float:
        return self.stator_z0 - 0.5

    @property
    def magnet_h(self) -> float:
        return self.stator_h + 1.0

    @property
    def shaft_z0(self) -> float:
        return 0.3

    @property
    def circlip_z0(self) -> float:
        return self.shaft_z0 + 0.1

    @property
    def bearing_bottom_z0(self) -> float:
        return self.circlip_z0 + 0.6

    @property
    def bearing_top_z0(self) -> float:
        return self.stator_z1 - self.bearing_w

    def __post_init__(self):
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if f.type is float and value <= 0:
                raise ValueError(f"{f.name} must be positive, got {value}")
        if self.detail not in DETAILS:
            raise ValueError(f"detail must be one of {', '.join(DETAILS)}")
        if self.slot_count < 3 or self.slot_count % 3:
            raise ValueError("slot_count must be a multiple of 3")
        if self.pole_count < 2 or self.pole_count % 2:
            raise ValueError("pole_count must be even")
        if not 0 < self.magnet_fill < 1:
            raise ValueError("magnet_fill must be between 0 and 1")
        if self.vent_count < 0:
            raise ValueError("vent_count must be 0 or more")
        if self.mount_count < 1:
            raise ValueError("mount_count must be 1 or more")

        # Stator and windings
        if self.bearing_od <= self.shaft_d + 1.0:
            raise ValueError("bearing_od is too small for shaft_d")
        if self.stator_bore <= self.bearing_od + 1.0:
            raise ValueError("stator_bore is too small for bearing_od")
        if self.tip_in_r <= self.yoke_r + 1.0:
            raise ValueError("no room for teeth: check stator_d, stator_bore, yoke_t, tip_t")
        half_pitch = math.pi / self.slot_count
        if math.atan2(self.coil_hw, self.yoke_r + 0.1) >= half_pitch * 0.95:
            raise ValueError("windings of next teeth collide: reduce tooth_w or coil_t")
        if math.sqrt(self.tip_in_r**2 - self.coil_hw**2) - 0.1 <= self.yoke_r + 0.6:
            raise ValueError("windings do not fit under the tooth tips")
        if self.slot_open >= 2 * math.pi * self.stator_d / 2 / self.slot_count - 0.5:
            raise ValueError("slot_open is too large")
        if self.coil_t + self.clearance - 0.5 < self.gap:
            raise ValueError("magnets reach below the bell: increase clearance")

        # Bearings and circlip
        if self.bearing_bottom_z0 + self.bearing_w >= self.bearing_top_z0:
            raise ValueError("bearings overlap: increase stator_h or reduce bearing_w")
        if self.shaft_d + 1.5 >= self.bearing_od:
            raise ValueError("circlip does not fit in the bearing bore")

        # Bell vents
        if self.vent_count > 0:
            if self.vent_pcd + self.vent_d >= 2 * self.magnet_out_r:
                raise ValueError("vent holes cut into the bell side wall")
            if self.vent_pcd - self.vent_d <= self.boss_d:
                raise ValueError("vent holes overlap the shaft boss")

        # Base
        if self.mount_pcd + self.mount_hole_d >= self.base_d:
            raise ValueError("mount holes cut into the base side")
        if self.mount_pcd - self.mount_hole_d <= self.bearing_od + 1.0:
            raise ValueError("mount holes overlap the bearing bore")
        if self.mount_hole_depth >= self.base_h:
            raise ValueError("mount_hole_depth must be smaller than base_h")
        if self.fillet_r >= min(self.base_h, self.bell_top_t) - 0.01:
            raise ValueError("fillet_r is too large for base_h or bell_top_t")


def ring(od: float, id_: float, h: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z0).circle(od / 2).circle(id_ / 2).extrude(h)


def polar_copies(shape: cq.Shape, count: int, start: float = 0.0) -> list[cq.Shape]:
    step = 360.0 / count
    return [shape.rotate(ORIGIN, Z_AXIS, start + k * step) for k in range(count)]


def make_base(p: MotorParams) -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .circle(p.base_d / 2)
        .extrude(p.base_h)
        .faces(">Z")
        .edges()
        .fillet(p.fillet_r)
    )
    # The stator tube holds the stator on its outside and the bearings inside.
    tube = (
        cq.Workplane("XY")
        .workplane(offset=p.base_h - 0.1)
        .circle(p.stator_bore / 2)
        .extrude(p.stator_z1 - p.base_h + 0.1)
    )
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.5)
        .circle(p.bearing_od / 2)
        .extrude(p.stator_z1 + 1.0)
    )
    holes = (
        cq.Workplane("XY")
        .polarArray(p.mount_pcd / 2, 0, 360, p.mount_count)
        .circle(p.mount_hole_d / 2)
        .extrude(p.mount_hole_depth)
    )
    return plate.union(tube).cut(bore).cut(holes)


def make_stator(p: MotorParams) -> cq.Workplane:
    wp = cq.Workplane("XY").workplane(offset=p.stator_z0)
    h = p.stator_h
    yoke = wp.circle(p.yoke_r).circle(p.stator_bore / 2).extrude(h)
    # polarArray rotates each point, so local x of each rect points outward.
    tooth_len = p.tip_in_r - p.yoke_r + 0.4
    teeth = (
        wp.polarArray((p.yoke_r + p.tip_in_r) / 2, 0, 360, p.slot_count)
        .rect(tooth_len, p.tooth_w)
        .extrude(h)
    )
    tips = wp.circle(p.stator_d / 2).circle(p.tip_in_r).extrude(h)
    openings = (
        wp.polarArray(p.stator_d / 2 - p.tip_t / 2, 180.0 / p.slot_count, 360, p.slot_count)
        .rect(p.tip_t + 1.0, p.slot_open)
        .extrude(h)
    )
    return yoke.union(teeth).union(tips.cut(openings))


def make_windings(p: MotorParams) -> cq.Shape:
    """One coil around each tooth. The coil ends stick out above and below the stator."""
    x0 = p.yoke_r + 0.1
    x1 = math.sqrt(p.tip_in_r**2 - p.coil_hw**2) - 0.1
    outer = (
        cq.Workplane("XY")
        .box(x1 - x0, 2 * p.coil_hw, p.stator_h + 2 * p.coil_t, centered=(False, True, False))
        .edges("|X")
        .fillet(p.coil_t * 0.9)
        .translate((x0, 0, p.stator_z0 - p.coil_t))
    )
    tooth = (
        cq.Workplane("XY")
        .box(x1 - x0 + 1.0, p.tooth_w, p.stator_h, centered=(False, True, False))
        .translate((x0 - 0.5, 0, p.stator_z0))
    )
    coil = outer.cut(tooth).val()
    return cq.Compound.makeCompound(polar_copies(coil, p.slot_count))


def make_magnets(p: MotorParams) -> tuple[cq.Shape, cq.Shape]:
    """Arc magnets inside the bell. Returns (north poles, south poles)."""
    arc = 360.0 / p.pole_count * p.magnet_fill
    base = cq.Vector(0, 0, p.magnet_z0)
    outer = cq.Solid.makeCylinder(p.magnet_out_r, p.magnet_h, base, Z_AXIS, arc)
    inner = cq.Solid.makeCylinder(p.magnet_in_r, p.magnet_h, base, Z_AXIS, arc)
    magnet = outer.cut(inner).rotate(ORIGIN, Z_AXIS, -arc / 2)
    magnets = polar_copies(magnet, p.pole_count)
    return (
        cq.Compound.makeCompound(magnets[0::2]),
        cq.Compound.makeCompound(magnets[1::2]),
    )


def make_bell(p: MotorParams) -> cq.Workplane:
    plane = cq.Workplane("XY").workplane(offset=p.bell_z0)
    bell_h = p.bell_top_z - p.bell_z0
    bell = (
        plane.circle(p.bell_d / 2)
        .extrude(bell_h)
        .faces(">Z")
        .edges()
        .fillet(p.fillet_r)
    )
    inner = plane.workplane(offset=-0.1).circle(p.magnet_out_r).extrude(bell_h - p.bell_top_t + 0.1)
    boss = (
        cq.Workplane("XY")
        .workplane(offset=p.bell_top_z - 0.1)
        .circle(p.boss_d / 2)
        .extrude(p.boss_h + 0.1)
    )
    shaft_hole = (
        cq.Workplane("XY")
        .workplane(offset=p.bell_top_z - p.bell_top_t - 0.1)
        .circle(p.shaft_d / 2)
        .extrude(p.bell_top_t + p.boss_h + 0.2)
    )
    bell = bell.cut(inner).union(boss).cut(shaft_hole)
    if p.vent_count > 0:
        # Rotate by half a pitch so vents sit between the mount-hole directions.
        vents = (
            cq.Workplane("XY")
            .workplane(offset=p.bell_top_z - p.bell_top_t - 0.1)
            .polarArray(p.vent_pcd / 2, 180.0 / p.vent_count, 360, p.vent_count)
            .circle(p.vent_d / 2)
            .extrude(p.bell_top_t + 0.2)
        )
        bell = bell.cut(vents)
    return bell


def make_shaft(p: MotorParams) -> cq.Workplane:
    z0 = p.shaft_z0
    top = p.bell_top_z + p.shaft_len
    if not p.prop_nut_thread:
        return (
            cq.Workplane("XY")
            .workplane(offset=z0)
            .circle(p.shaft_d / 2)
            .extrude(top - z0)
            .faces(">Z")
            .chamfer(0.3)
        )
    # Upper part of the shaft is drawn at the thread minor diameter.
    thread_len = 0.6 * p.shaft_len
    thread_d = p.shaft_d - 0.6
    plain = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .circle(p.shaft_d / 2)
        .extrude(top - thread_len - z0)
    )
    thread = (
        cq.Workplane("XY")
        .workplane(offset=top - thread_len)
        .circle(thread_d / 2)
        .extrude(thread_len)
        .faces(">Z")
        .chamfer(0.3)
    )
    return plain.union(thread)


def make_bearing(p: MotorParams, z0: float) -> cq.Workplane:
    """Plain ring with a shallow groove on each face between the races."""
    bearing = ring(p.bearing_od, p.shaft_d, p.bearing_w, z0)
    race_t = (p.bearing_od - p.shaft_d) / 2 * 0.3
    groove_od = p.bearing_od - 2 * race_t
    groove_id = p.shaft_d + 2 * race_t
    for gz in (z0 - 0.1, z0 + p.bearing_w - 0.3):
        bearing = bearing.cut(ring(groove_od, groove_id, 0.4, gz))
    return bearing


def make_circlip(p: MotorParams) -> cq.Workplane:
    return ring(p.shaft_d + 1.5, p.shaft_d, 0.5, p.circlip_z0)


def make_simple_base(p: MotorParams) -> cq.Workplane:
    """Base plate with a solid core up to the bell top, in place of the stator and bearings.
    A recess in the bottom face shows the shaft end."""
    plate = (
        cq.Workplane("XY")
        .circle(p.base_d / 2)
        .extrude(p.base_h)
        .faces(">Z")
        .edges()
        .fillet(p.fillet_r)
    )
    core = (
        cq.Workplane("XY")
        .workplane(offset=p.base_h - 0.1)
        .circle(p.stator_bore / 2)
        .extrude(p.bell_top_z - p.bell_top_t - p.base_h + 0.1)
    )
    shaft_bore = cq.Workplane("XY").workplane(offset=-0.5).circle(p.shaft_d / 2).extrude(p.bell_top_z + 1.0)
    recess = cq.Workplane("XY").workplane(offset=-0.1).circle(p.shaft_d / 2 + 0.75).extrude(p.circlip_z0 + 0.6)
    holes = (
        cq.Workplane("XY")
        .polarArray(p.mount_pcd / 2, 0, 360, p.mount_count)
        .circle(p.mount_hole_d / 2)
        .extrude(p.mount_hole_depth)
    )
    return plate.union(core).cut(shaft_bore).cut(recess).cut(holes)


def build_motor(p: MotorParams) -> cq.Assembly:
    assy = cq.Assembly(name="drone_motor")
    if p.detail == "simple":
        assy.add(make_simple_base(p), name="base", color=cq.Color(0.20, 0.20, 0.22))
        assy.add(make_bell(p), name="bell", color=cq.Color(0.80, 0.10, 0.10))
        assy.add(make_shaft(p), name="shaft", color=cq.Color(0.80, 0.80, 0.82))
        return assy
    magnets_n, magnets_s = make_magnets(p)
    assy.add(make_base(p), name="base", color=cq.Color(0.20, 0.20, 0.22))
    assy.add(make_stator(p), name="stator", color=cq.Color(0.45, 0.45, 0.48))
    assy.add(make_windings(p), name="windings", color=cq.Color(0.72, 0.45, 0.20))
    assy.add(make_bell(p), name="bell", color=cq.Color(0.80, 0.10, 0.10))
    assy.add(magnets_n, name="magnets_n", color=cq.Color(0.60, 0.20, 0.20))
    assy.add(magnets_s, name="magnets_s", color=cq.Color(0.20, 0.30, 0.60))
    assy.add(make_shaft(p), name="shaft", color=cq.Color(0.80, 0.80, 0.82))
    assy.add(make_bearing(p, p.bearing_bottom_z0), name="bearing_bottom", color=cq.Color(0.70, 0.70, 0.72))
    assy.add(make_bearing(p, p.bearing_top_z0), name="bearing_top", color=cq.Color(0.70, 0.70, 0.72))
    assy.add(make_circlip(p), name="circlip", color=cq.Color(0.10, 0.10, 0.10))
    return assy


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a drone motor STEP assembly.")
    for f in dataclasses.fields(MotorParams):
        flag = "--" + f.name.replace("_", "-")
        if f.type is bool:
            parser.add_argument(flag, action=argparse.BooleanOptionalAction, default=f.default)
        else:
            parser.add_argument(flag, type=f.type, default=f.default, choices=f.metadata.get("choices"),
                                help=f"default: {f.default}")
    parser.add_argument("-o", "--output", default="drone_motor.step", help="STEP output path")
    args = vars(parser.parse_args())
    output = args.pop("output")

    try:
        params = MotorParams(**args)
    except ValueError as e:
        parser.error(str(e))

    assy = build_motor(params)
    assy.export(output)
    bb = assy.toCompound().BoundingBox()
    print(f"Wrote {output}")
    print(f"Parts: {', '.join(child.name for child in assy.children)}")
    print(f"Size: X {bb.xlen:.2f}  Y {bb.ylen:.2f}  Z {bb.zlen:.2f} mm")


if __name__ == "__main__":
    main()
