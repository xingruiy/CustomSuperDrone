"""Drone motor with propeller and prop nut, as one STEP assembly.

Uses the full motor from drone_motor_full.py and the propeller from prop.py.
The prop hub sits on the shaft boss of the bell. A hex nut on the shaft holds it.

The Z axis is the motor axis. Z = 0 is the motor mounting face. All units are mm.
Motor flags have no prefix. Prop flags start with --prop- (for example --prop-pitch).

Usage:
    uv run python motor_prop.py
    uv run python motor_prop.py --spec 5x4.3x3 --prop-direction cw -o motor_prop_cw.step
    uv run python motor_prop.py --stator-h 6 --spec 5.1x3.6x3
"""

import argparse
import dataclasses

import cadquery as cq

from drone_motor_full import FullMotorParams, build_motor
from prop import PropParams, build_prop, parse_spec

PROP_PREFIX = "prop_"


def nut_size(p: FullMotorParams) -> tuple[float, float]:
    """Nut (across flats, height). Close to an M5 nyloc nut for a 5 mm shaft."""
    return 1.6 * p.shaft_d, p.shaft_d


def prop_z(m: FullMotorParams) -> float:
    """Z of the prop hub bottom: the top of the shaft boss on the bell."""
    return m.bell_top_z + m.boss_h


def make_nut(m: FullMotorParams, z0: float) -> cq.Workplane:
    af, h = nut_size(m)
    thread_d = m.shaft_d - 0.6 if m.prop_nut_thread else m.shaft_d
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .polygon(6, af / 0.8660254)  # corner-to-corner diameter
        .extrude(h)
        .edges("not |Z")
        .chamfer(0.4)
        .faces(">Z")
        .workplane()
        .hole(thread_d)
    )


def check_fit(m: FullMotorParams, pp: PropParams, prop: cq.Workplane) -> None:
    z0 = prop_z(m)
    _, nut_h = nut_size(m)
    shaft_top = m.bell_top_z + m.shaft_len
    if z0 + pp.hub_h + nut_h > shaft_top:
        raise ValueError(
            f"shaft is too short: prop hub and nut need "
            f"{m.boss_h + pp.hub_h + nut_h:.1f} mm above the bell top, "
            f"shaft_len is {m.shaft_len}"
        )
    # The blades must not touch the bell.
    bell_zone = cq.Workplane("XY").circle(m.bell_d / 2 + 0.5).extrude(m.bell_top_z + 0.3)
    hit = prop.translate((0, 0, z0)).val().intersect(bell_zone.val()).Volume()
    if hit > 1e-3:
        raise ValueError("prop blades hit the bell: increase --shaft-len or reduce --prop-max-twist")


def build_motor_prop(m: FullMotorParams, pp: PropParams) -> cq.Assembly:
    prop = build_prop(pp)
    check_fit(m, pp, prop)
    z0 = prop_z(m)
    assy = cq.Assembly(name="motor_prop")
    assy.add(build_motor(m), name="motor")
    assy.add(prop, name="prop", loc=cq.Location((0, 0, z0)), color=cq.Color(0.95, 0.55, 0.10))
    assy.add(make_nut(m, z0 + pp.hub_h), name="prop_nut", color=cq.Color(0.15, 0.35, 0.85))
    return assy


def add_param_args(parser: argparse.ArgumentParser, cls, prefix: str = "") -> None:
    """One flag per dataclass field. Default None marks flags the user did not give."""
    for f in dataclasses.fields(cls):
        flag = "--" + (prefix + f.name).replace("_", "-")
        dest = prefix + f.name
        if f.type is bool:
            parser.add_argument(flag, dest=dest, action=argparse.BooleanOptionalAction, default=None,
                                help=f"default: {f.default}")
        else:
            default = f"{f.default:.4g}" if f.type is float else f.default
            parser.add_argument(flag, dest=dest, type=f.type, default=None,
                                choices=f.metadata.get("choices"), help=f"default: {default}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a drone motor + prop STEP assembly.")
    parser.add_argument(
        "--spec",
        help="prop size in inches as DIAMETERxPITCH[xBLADES], e.g. 5.1x4.3x3. --prop-* flags override it.",
    )
    motor_group = parser.add_argument_group("motor")
    add_param_args(motor_group, FullMotorParams)
    prop_group = parser.add_argument_group("prop")
    add_param_args(prop_group, PropParams, PROP_PREFIX)
    parser.add_argument("-o", "--output", default="motor_prop.step", help="STEP output path")
    args = vars(parser.parse_args())
    output = args.pop("output")
    spec = args.pop("spec")

    # Split by field name: the motor field prop_nut_thread also starts with "prop_".
    motor_names = {f.name for f in dataclasses.fields(FullMotorParams)}
    prop_names = {PROP_PREFIX + f.name for f in dataclasses.fields(PropParams)}
    motor_values = {k: v for k, v in args.items() if v is not None and k in motor_names}
    prop_values = {
        k.removeprefix(PROP_PREFIX): v for k, v in args.items() if v is not None and k in prop_names
    }
    try:
        m = FullMotorParams(**motor_values)
        values = parse_spec(spec) if spec else {}
        values["bore_d"] = m.shaft_d  # the prop fits the motor shaft
        values.update(prop_values)
        if values["bore_d"] != m.shaft_d:
            raise ValueError(f"--prop-bore-d ({values['bore_d']}) must match --shaft-d ({m.shaft_d})")
        pp = PropParams(**values)
        assy = build_motor_prop(m, pp)
    except ValueError as e:
        parser.error(str(e))

    assy.export(output)
    bb = assy.toCompound().BoundingBox()
    print(f"Wrote {output}")
    print(f"Prop: {pp.diameter / 25.4:.2f} x {pp.pitch / 25.4:.2f} in, {pp.blade_count} blades, {pp.direction}")
    print(f"Size: X {bb.xlen:.2f}  Y {bb.ylen:.2f}  Z {bb.zlen:.2f} mm")


if __name__ == "__main__":
    main()
