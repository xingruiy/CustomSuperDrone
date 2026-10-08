"""Parametric outer-shape model of a brushless outrunner drone motor.

The Z axis is the motor axis. Z = 0 is the base bottom face (the mounting face).
All units are mm. Default values match a typical 2207 motor.

Usage:
    uv run python drone_motor.py
    uv run python drone_motor.py --bell-d 30 --vent-count 5 -o m2306.step
"""

import argparse
import dataclasses
from dataclasses import dataclass

import cadquery as cq


@dataclass
class MotorParams:
    bell_d: float = 27.9  # bell outer diameter
    bell_h: float = 12.0  # bell height (side wall)
    bell_top_t: float = 1.5  # bell top and side wall thickness
    vent_count: int = 6  # vent holes in bell top (0 = none)
    vent_d: float = 5.0  # vent hole diameter
    vent_pcd: float = 17.0  # vent hole pitch circle diameter
    base_d: float = 27.0  # base diameter
    base_h: float = 4.0  # base height
    gap: float = 0.5  # axial gap between base and bell
    mount_count: int = 4  # mount holes in base
    mount_pcd: float = 16.0  # mount hole pitch circle diameter
    mount_hole_d: float = 3.0  # mount hole diameter (M3)
    mount_hole_depth: float = 3.5  # mount hole depth
    shaft_d: float = 5.0  # shaft diameter
    shaft_len: float = 14.0  # shaft length above bell top
    prop_nut_thread: bool = True  # smaller thread diameter at shaft tip
    fillet_r: float = 0.8  # edge fillet radius on bell and base

    @property
    def hub_d(self) -> float:
        """Diameter of the hub that joins the base and the bell."""
        return self.shaft_d + 2.0

    @property
    def circlip_h(self) -> float:
        """Depth of the circlip recess under the shaft."""
        return min(1.0, self.base_h / 4)

    def __post_init__(self):
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if f.type is float and value <= 0:
                raise ValueError(f"{f.name} must be positive, got {value}")
        if self.vent_count < 0:
            raise ValueError("vent_count must be 0 or more")
        if self.mount_count < 1:
            raise ValueError("mount_count must be 1 or more")

        wall = self.bell_top_t
        if 2 * wall >= self.bell_d:
            raise ValueError("bell_top_t is too large for bell_d")
        if wall >= self.bell_h:
            raise ValueError("bell_top_t must be smaller than bell_h")
        if self.hub_d >= self.bell_d - 2 * wall:
            raise ValueError("shaft_d is too large for the bell")
        if self.vent_count > 0:
            if self.vent_pcd + self.vent_d >= self.bell_d - 2 * wall:
                raise ValueError("vent holes cut into the bell side wall")
            if self.vent_pcd - self.vent_d <= self.hub_d:
                raise ValueError("vent holes overlap the shaft hub")

        if self.mount_pcd + self.mount_hole_d >= self.base_d:
            raise ValueError("mount holes cut into the base side")
        if self.mount_pcd - self.mount_hole_d <= self.shaft_d + 1.0:
            raise ValueError("mount holes overlap the center recess")
        if self.mount_hole_depth >= self.base_h:
            raise ValueError("mount_hole_depth must be smaller than base_h")
        if self.fillet_r >= min(self.base_h, wall, self.gap + self.bell_h) - 0.01:
            raise ValueError("fillet_r is too large for base_h or bell_top_t")

    @property
    def bell_z0(self) -> float:
        return self.base_h + self.gap

    @property
    def bell_top_z(self) -> float:
        return self.bell_z0 + self.bell_h


def make_base(p: MotorParams) -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .circle(p.base_d / 2)
        .extrude(p.base_h)
        .faces(">Z")
        .edges()
        .fillet(p.fillet_r)
    )
    # Blind mount holes and the circlip recess, cut from the bottom face.
    holes = (
        cq.Workplane("XY")
        .polarArray(p.mount_pcd / 2, 0, 360, p.mount_count)
        .circle(p.mount_hole_d / 2)
        .extrude(p.mount_hole_depth)
    )
    recess = cq.Workplane("XY").circle(p.shaft_d / 2 + 0.5).extrude(p.circlip_h)
    return base.cut(holes).cut(recess)


def make_bell(p: MotorParams) -> cq.Workplane:
    wall = p.bell_top_t
    plane = cq.Workplane("XY").workplane(offset=p.bell_z0)
    bell = (
        plane.circle(p.bell_d / 2)
        .extrude(p.bell_h)
        .faces(">Z")
        .edges()
        .fillet(p.fillet_r)
    )
    inner = plane.circle(p.bell_d / 2 - wall).extrude(p.bell_h - wall)
    bell = bell.cut(inner)
    if p.vent_count > 0:
        # Rotate by half a pitch so vents sit between the mount-hole directions.
        start = 180.0 / p.vent_count
        vents = (
            cq.Workplane("XY")
            .workplane(offset=p.bell_top_z - wall - 0.1)
            .polarArray(p.vent_pcd / 2, start, 360, p.vent_count)
            .circle(p.vent_d / 2)
            .extrude(wall + 0.2)
        )
        bell = bell.cut(vents)
    return bell


def make_hub(p: MotorParams) -> cq.Workplane:
    """Solid hub from the base top to the bell top. It makes the model one solid."""
    return (
        cq.Workplane("XY")
        .workplane(offset=p.base_h - 0.1)
        .circle(p.hub_d / 2)
        .extrude(p.gap + p.bell_h + 0.1)
    )


def make_shaft(p: MotorParams) -> cq.Workplane:
    z0 = p.circlip_h
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


def build_motor(p: MotorParams) -> cq.Workplane:
    return make_base(p).union(make_bell(p)).union(make_hub(p)).union(make_shaft(p))


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a drone motor STEP model.")
    for f in dataclasses.fields(MotorParams):
        flag = "--" + f.name.replace("_", "-")
        if f.type is bool:
            parser.add_argument(flag, action=argparse.BooleanOptionalAction, default=f.default)
        else:
            parser.add_argument(flag, type=f.type, default=f.default, help=f"default: {f.default}")
    parser.add_argument("-o", "--output", default="drone_motor.step", help="STEP output path")
    args = vars(parser.parse_args())
    output = args.pop("output")

    try:
        params = MotorParams(**args)
    except ValueError as e:
        parser.error(str(e))

    model = build_motor(params)
    cq.exporters.export(model, output)
    bb = model.val().BoundingBox()
    print(f"Wrote {output}")
    print(f"Size: X {bb.xlen:.2f}  Y {bb.ylen:.2f}  Z {bb.zlen:.2f} mm")


if __name__ == "__main__":
    main()
