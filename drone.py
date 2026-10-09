"""Full FPV quadcopter assembly, exported as a colored STEP file.

Parts:
- frame: true-X unibody bottom plate with arms, top plate, standoffs, camera cage
- 4 x motor + prop + prop nut (from motor_prop.py)
- stack: fake 4-in-1 ESC and flight controller (from fcu.py) with spacers, screws, nuts
- FPV camera, battery and battery strap

Front is +X, left is +Y, up is +Z. Z = 0 is the bottom face of the bottom plate.
Prop directions follow the Betaflight default ("props in"): front-right and
rear-left turn ccw, front-left and rear-right turn cw (viewed from above).
All units are mm.

Usage:
    uv run python drone.py
    uv run python drone.py --props-out --cam-angle 40 -o drone_out.step
    uv run python drone.py --wheelbase 250 --spec 6x4x3 --bat-l 85
"""

import argparse
import dataclasses
import math
import random
from dataclasses import dataclass

import cadquery as cq

from drone_motor import MotorParams
from fcu import FCParams, Layout, block, build_fc, chip, jst_sh, mount_hardware, ring, to_assembly
from motor_prop import build_motor_prop
from prop import PropParams, parse_spec

CU = 0.035  # copper ring thickness on the stack boards
SCREW_D = 2.9  # M3 stack screws
NUT_AF, NUT_H = 5.5, 2.4  # M3 nut
COLORS = {
    "frame": (0.12, 0.12, 0.13),
    "standoffs": (0.75, 0.20, 0.10),
    "spacers": (0.90, 0.90, 0.85),
    "screws": (0.60, 0.60, 0.62),
    "camera": (0.08, 0.08, 0.08),
    "camera_glass": (0.10, 0.20, 0.45),
    "battery": (0.15, 0.40, 0.80),
    "strap": (0.85, 0.10, 0.10),
}
# (name, arm angle in degrees, prop direction with props in)
MOTORS = [("fr", -45, "ccw"), ("fl", 45, "cw"), ("rl", 135, "ccw"), ("rr", -135, "cw")]


@dataclass
class DroneParams:
    # Frame
    wheelbase: float = 225.0  # diagonal motor-to-motor distance
    arm_w: float = 14.0
    arm_slots: bool = True  # weight-saving slot in each arm
    motor_pad_d: float = 25.0  # round pad under each motor
    plate_t: float = 5.0  # bottom plate (and arms) thickness
    top_t: float = 2.0  # top plate thickness
    body_l: float = 90.0  # center body length (X)
    body_w: float = 42.0  # center body width (Y)
    standoff_h: float = 28.0  # space between bottom and top plate
    standoff_d: float = 5.0
    # Stack
    spacer_low: float = 4.0  # spacer between bottom plate and ESC grommets
    spacer_mid: float = 6.0  # spacer between ESC and FC grommets
    # Camera
    cam_size: float = 19.0  # micro camera body width and height
    cam_depth: float = 12.0  # body depth behind the lens
    cam_angle: float = 30.0  # up tilt in degrees
    # Battery
    bat_l: float = 75.0
    bat_w: float = 35.0
    bat_h: float = 35.0
    bat_x: float = -5.0  # battery center X
    # Props
    spec: str = "5.1x4.3x3"  # prop size in inches: DIAMETERxPITCH[xBLADES]
    props_out: bool = False  # reverse all prop directions
    seed: int = 1  # random layout of the small parts on the FC and ESC

    @property
    def arm_r(self) -> float:
        return self.wheelbase / 2

    @property
    def standoff_points(self) -> list[tuple[float, float]]:
        x, y = self.body_l / 2 - 6.0, self.body_w / 2 - 5.0
        return [(x, y), (-x, y), (-x, -y), (x, -y)]

    @property
    def top_z(self) -> float:
        """Z of the top plate bottom face."""
        return self.plate_t + self.standoff_h

    @property
    def cam_x(self) -> float:
        """X of the camera front face (lens base)."""
        return self.body_l / 2 - 6.0

    def __post_init__(self):
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if f.type is float and value <= 0 and f.name not in ("bat_x", "cam_angle"):
                raise ValueError(f"{f.name} must be positive, got {value}")
        parse_spec(self.spec)
        if not 0 <= self.cam_angle < 60:
            raise ValueError("cam_angle must be between 0 and 60 degrees")
        if self.bat_w / 2 + 2.0 > self.body_w / 2:
            raise ValueError("bat_w is too large for body_w (the strap needs slots in the top plate)")
        if self.cam_size / 2 + 2.5 > self.body_w / 2 - 5.0 - self.standoff_d / 2:
            raise ValueError("camera cage hits the front standoffs: increase body_w or reduce cam_size")


def motor_positions(d: DroneParams) -> list[tuple[str, float, float, float, str]]:
    out = []
    for name, angle, direction in MOTORS:
        if d.props_out:
            direction = "cw" if direction == "ccw" else "ccw"
        a = math.radians(angle)
        out.append((name, angle, d.arm_r * math.cos(a), d.arm_r * math.sin(a), direction))
    return out


def rect_distance(px: float, py: float, x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(max(x0 - px, 0, px - x1), max(y0 - py, 0, py - y1))


def check_layout(d: DroneParams, m: MotorParams, pp: PropParams, fc: FCParams) -> None:
    prop_r = pp.diameter / 2
    side = d.wheelbase / math.sqrt(2)
    if side < 2 * prop_r + 2.0:
        raise ValueError(f"props overlap: wheelbase {d.wheelbase} is too small for {d.spec} props")
    body = (-d.body_l / 2, -d.body_w / 2, d.body_l / 2, d.body_w / 2)
    battery = (d.bat_x - d.bat_l / 2, -d.bat_w / 2, d.bat_x + d.bat_l / 2, d.bat_w / 2)
    for name, _, x, y, _ in motor_positions(d):
        if rect_distance(x, y, *body) < prop_r + 1.0:
            raise ValueError(f"prop {name} hits the frame body: increase wheelbase or reduce body size")
        if rect_distance(x, y, *battery) < prop_r + 1.0:
            raise ValueError(f"prop {name} hits the battery: increase wheelbase or change battery size")
    if d.arm_r - d.motor_pad_d / 2 < math.hypot(d.body_l, d.body_w) / 2 * 0.5:
        raise ValueError("motor pads are too close to the body: increase wheelbase")
    if m.mount_pcd + m.mount_hole_d + 2.0 > d.motor_pad_d:
        raise ValueError("motor_pad_d is too small for the motor mount holes")
    if d.bat_l / 2 + abs(d.bat_x) > d.body_l / 2 + 25.0:
        raise ValueError("battery overhangs the top plate too much")
    if fc.board_w / 2 + 1.0 > d.body_l / 2 - 6.0 - d.standoff_d / 2 or fc.board_l / 2 > d.body_w / 2:
        raise ValueError("the stack boards do not fit in the frame body")


# Frame


def plate_outline(d: DroneParams, t: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(d.body_l, d.body_w, t, centered=(True, True, False))
        .edges("|Z")
        .fillet(min(6.0, d.body_w / 2 - 0.5))
    )


def arm_exit_r(d: DroneParams, angle: float) -> float:
    """Distance from the center to where the arm center line leaves the body."""
    a = math.radians(angle)
    return min(d.body_l / 2 / abs(math.cos(a)), d.body_w / 2 / abs(math.sin(a)))


def make_bottom_plate(d: DroneParams, m: MotorParams, fc: FCParams) -> cq.Workplane:
    plate = plate_outline(d, d.plate_t)
    for _, angle, x, y, _ in motor_positions(d):
        arm = (
            cq.Workplane("XY")
            .transformed(rotate=(0, 0, angle))
            .center(d.arm_r / 2, 0)
            .slot2D(d.arm_r, d.arm_w)
            .extrude(d.plate_t)
        )
        pad = cq.Workplane("XY").center(x, y).circle(d.motor_pad_d / 2).extrude(d.plate_t)
        plate = plate.union(arm).union(pad)

    holes = []
    for _, angle, x, y, _ in motor_positions(d):
        for k in range(m.mount_count):
            a = math.radians(angle + 360.0 * k / m.mount_count)
            holes.append((x + m.mount_pcd / 2 * math.cos(a), y + m.mount_pcd / 2 * math.sin(a)))
    cutters = (
        cq.Workplane("XY").workplane(offset=-1)
        .pushPoints(holes).circle(m.mount_hole_d / 2 + 0.1)
        .pushPoints([(x, y) for _, _, x, y, _ in motor_positions(d)]).circle(4.0)
        .pushPoints(fc.hole_points).circle(1.6)
        .pushPoints(d.standoff_points).circle(1.6)
        .extrude(d.plate_t + 2)
    )
    plate = plate.cut(cutters)

    if d.arm_slots:
        for _, angle, _, _, _ in motor_positions(d):
            r0 = arm_exit_r(d, angle) + 6.0
            r1 = d.arm_r - d.motor_pad_d / 2 - 3.0
            if r1 - r0 > 2 * d.arm_w:
                slot = (
                    cq.Workplane("XY").workplane(offset=-1)
                    .transformed(rotate=(0, 0, angle))
                    .center((r0 + r1) / 2, 0)
                    .slot2D(r1 - r0, d.arm_w - 7.0)
                    .extrude(d.plate_t + 2)
                )
                plate = plate.cut(slot)
    return plate


def make_top_plate(d: DroneParams) -> cq.Workplane:
    plate = plate_outline(d, d.top_t).translate((0, 0, d.top_z))
    cutters = (
        cq.Workplane("XY").workplane(offset=d.top_z - 1)
        .pushPoints(d.standoff_points).circle(1.6)
        # Strap slots next to the battery sides.
        .pushPoints([(d.bat_x, d.bat_w / 2 + 0.5), (d.bat_x, -d.bat_w / 2 - 0.5)]).rect(20.6, 1.6)
        .extrude(d.top_t + 2)
    )
    return plate.cut(cutters)


def make_camera_cage(d: DroneParams) -> cq.Shape:
    """Two side plates next to the camera."""
    y = d.cam_size / 2 + 0.3 + 1.0
    return cq.Compound.makeCompound([
        cq.Solid.makeBox(20.0, 2.0, d.standoff_h, cq.Vector(d.cam_x - 18.0, sy * y - 1.0, d.plate_t))
        for sy in (1, -1)
    ])


def make_standoffs(d: DroneParams) -> cq.Shape:
    return cq.Compound.makeCompound([
        cq.Solid.makeCylinder(d.standoff_d / 2, d.standoff_h, cq.Vector(x, y, d.plate_t))
        for x, y in d.standoff_points
    ])


# Electronics stack


def make_esc(fc: FCParams, seed: int) -> cq.Assembly:
    """Fake 4-in-1 ESC on the same mount pattern as the FC."""
    p = dataclasses.replace(fc, connectors="", passives_top=0, passives_bottom=0)
    rng = random.Random(seed + 1000)
    lay = Layout(p)
    mount_hardware(p, lay)
    w, l = p.board_w / 2, p.board_l / 2
    # Motor pads on the side edges, battery pads on the rear edge, FC plug on the front edge.
    for sx in (1, -1):
        for sy in (1, -1):
            for k in range(3):
                lay.place("motor_pad", "top", block("copper", 2.0, 2.4, 0.05), sx * (2.5 + 3.0 * k), sy * (l - 2.0))
    for sy in (1, -1):
        lay.place("battery_pad", "top", block("copper", 4.8, 7.0, 0.05), -(w - 3.2), sy * 5.0)
    lay.place_on_edge("jst_sh_8pin", "+x", jst_sh(8), 0)
    for side in ("top", "bottom"):
        for _ in range(12):
            lay.place_random(rng, "mosfet", side, block("ics", 5.0, 6.0, 1.0))
    for _ in range(4):
        lay.place_random(rng, "esc_mcu", "top", block("ics", 4.0, 4.0, 0.9))
    for side in ("top", "bottom"):
        for _ in range(20):
            lay.place_random(rng, "passive", side, chip(rng.choice(("caps", "resistors")), 1.0, 0.5, 0.35), tries=150)
    return to_assembly(p, lay, "esc")


def stack_z(d: DroneParams, fc: FCParams) -> tuple[float, float]:
    """Board bottom Z of the ESC and the FC."""
    g = fc.grommet_h + CU
    esc_z = d.plate_t + d.spacer_low + g
    fc_z = esc_z + fc.board_t + g + d.spacer_mid + g
    return esc_z, fc_z


def make_stack_hardware(d: DroneParams, fc: FCParams) -> tuple[cq.Shape, cq.Shape]:
    """(spacers, screws + nuts) for the four stack holes."""
    g = fc.grommet_h + CU
    esc_z, fc_z = stack_z(d, fc)
    nut_z = fc_z + fc.board_t + g
    spacers, metal = [], []
    for x, y in fc.hole_points:
        spacers.append(ring(5.0, 3.2, d.plate_t, d.spacer_low).translate((x, y, 0)))
        spacers.append(ring(5.0, 3.2, esc_z + fc.board_t + g, d.spacer_mid).translate((x, y, 0)))
        head = cq.Solid.makeCylinder(2.75, 2.0, cq.Vector(x, y, -2.0))
        shaft = cq.Solid.makeCylinder(SCREW_D / 2, nut_z + NUT_H + 0.5, cq.Vector(x, y, 0))
        nut = (
            cq.Workplane("XY").workplane(offset=nut_z).center(x, y)
            .polygon(6, NUT_AF / 0.8660254).circle(SCREW_D / 2).extrude(NUT_H).val()
        )
        metal += [head.fuse(shaft).clean(), nut]
    return cq.Compound.makeCompound(spacers), cq.Compound.makeCompound(metal)


# Camera and battery


def make_camera(d: DroneParams) -> tuple[cq.Shape, cq.Shape]:
    """(camera body + lens, lens glass). Built facing +X, then tilted up."""
    s = d.cam_size
    body = cq.Solid.makeBox(d.cam_depth, s, s, cq.Vector(-d.cam_depth, -s / 2, -s / 2))
    lens = cq.Solid.makeCylinder(7.0, 8.0, cq.Vector(0, 0, 0), cq.Vector(1, 0, 0))
    glass = cq.Solid.makeCylinder(5.0, 0.3, cq.Vector(8.0, 0, 0), cq.Vector(1, 0, 0))
    camera = body.fuse(lens).clean()

    def tilt(shape: cq.Shape) -> cq.Shape:
        return shape.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -d.cam_angle)

    # Center the tilted camera in height between the plates.
    bb = tilt(camera).BoundingBox()
    place = cq.Vector(d.cam_x, 0, d.plate_t + d.standoff_h / 2 - (bb.zmin + bb.zmax) / 2)
    return tilt(camera).translate(place), tilt(glass).translate(place)


def make_battery(d: DroneParams) -> tuple[cq.Shape, cq.Shape]:
    """(battery, strap). The strap goes around the battery and under the top plate."""
    z0 = d.top_z + d.top_t
    battery = (
        cq.Workplane("XY")
        .box(d.bat_l, d.bat_w, d.bat_h, centered=(True, True, False))
        .edges("|X").fillet(3.0)
        .translate((d.bat_x, 0, z0))
        .val()
    )
    t, width = 1.0, 20.0
    z_low, z_high = d.top_z - t, z0 + d.bat_h + t
    outer = cq.Solid.makeBox(width, d.bat_w + 2 * t, z_high - z_low,
                             cq.Vector(d.bat_x - width / 2, -d.bat_w / 2 - t, z_low))
    inner = cq.Solid.makeBox(width + 2, d.bat_w, z_high - z_low - 2 * t,
                             cq.Vector(d.bat_x - width / 2 - 1, -d.bat_w / 2, z_low + t))
    return battery, outer.cut(inner)


def check_fit(d: DroneParams, fc: FCParams, camera: cq.Shape) -> None:
    _, fc_z = stack_z(d, fc)
    stack_top = max(fc_z + fc.board_t + 3.3, fc_z + fc.board_t + fc.grommet_h + CU + NUT_H + 0.5)
    if stack_top > d.top_z - 0.5:
        raise ValueError(
            f"stack is {stack_top:.1f} mm tall and hits the top plate at {d.top_z:.1f} mm: "
            "increase standoff_h or reduce the spacers"
        )
    bb = camera.BoundingBox()
    if bb.zmin < d.plate_t + 0.3 or bb.zmax > d.top_z - 0.3:
        raise ValueError("camera does not fit between the plates: reduce cam_angle or increase standoff_h")
    if bb.xmin < fc.board_w / 2 + 1.0:
        raise ValueError("camera hits the stack: increase body_l")


def build_drone(d: DroneParams) -> cq.Assembly:
    m = MotorParams()
    fc = FCParams(seed=d.seed)
    base_prop = dict(parse_spec(d.spec), bore_d=m.shaft_d)
    props = {dr: PropParams(**base_prop, direction=dr) for dr in ("ccw", "cw")}
    check_layout(d, m, props["ccw"], fc)
    camera, glass = make_camera(d)
    check_fit(d, fc, camera)

    def color(name: str) -> cq.Color:
        return cq.Color(*COLORS[name])

    assy = cq.Assembly(name="drone")
    frame = cq.Assembly(name="frame")
    frame.add(make_bottom_plate(d, m, fc), name="bottom_plate", color=color("frame"))
    frame.add(make_top_plate(d), name="top_plate", color=color("frame"))
    frame.add(make_camera_cage(d), name="camera_cage", color=color("frame"))
    frame.add(make_standoffs(d), name="standoffs", color=color("standoffs"))
    assy.add(frame)

    units = {dr: build_motor_prop(m, pp) for dr, pp in props.items()}
    for name, angle, x, y, direction in motor_positions(d):
        loc = cq.Location(cq.Vector(x, y, d.plate_t), cq.Vector(0, 0, 1), angle)
        assy.add(units[direction], name=f"motor_{name}", loc=loc)

    esc_z, fc_z = stack_z(d, fc)
    spacers, metal = make_stack_hardware(d, fc)
    stack = cq.Assembly(name="stack")
    stack.add(make_esc(fc, d.seed), loc=cq.Location((0, 0, esc_z)))
    stack.add(build_fc(fc)[0], loc=cq.Location((0, 0, fc_z)))
    stack.add(spacers, name="spacers", color=color("spacers"))
    stack.add(metal, name="screws", color=color("screws"))
    assy.add(stack)

    battery, strap = make_battery(d)
    assy.add(camera, name="camera", color=color("camera"))
    assy.add(glass, name="camera_glass", color=color("camera_glass"))
    assy.add(battery, name="battery", color=color("battery"))
    assy.add(strap, name="strap", color=color("strap"))
    return assy


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a full FPV drone STEP assembly.")
    for f in dataclasses.fields(DroneParams):
        flag = "--" + f.name.replace("_", "-")
        if f.type is bool:
            parser.add_argument(flag, action=argparse.BooleanOptionalAction, default=f.default)
        else:
            parser.add_argument(flag, type=f.type, default=f.default, help=f"default: {f.default}")
    parser.add_argument("-o", "--output", default="drone.step", help="STEP output path")
    args = vars(parser.parse_args())
    output = args.pop("output")

    try:
        params = DroneParams(**args)
        assy = build_drone(params)
    except ValueError as e:
        parser.error(str(e))

    assy.export(output)
    bb = assy.toCompound().BoundingBox()
    print(f"Wrote {output}")
    print(f"Motors: " + ", ".join(f"{n} {dr}" for n, _, _, _, dr in motor_positions(params)))
    print(f"Size: X {bb.xlen:.1f}  Y {bb.ylen:.1f}  Z {bb.zlen:.1f} mm")


if __name__ == "__main__":
    main()
