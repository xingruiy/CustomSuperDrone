"""SUPER quadrotor (HKU MARS) as a multi-file STEP assembly.

The frame is the SUPER carbon fiber kit (hardware/CarbonFiber, from hku-mars/SUPER-Hardware). The
Livox Mid-360S sits on the top plate, inside a 3D printed guard (lidar_guard.py)
that screws to the same plate. The NUC 13 Pro board sits on the main
plate, between the main and top plates. The flight controller and ESC stack
hang under the main plate. The battery hangs under the battery plate in a
3D printed holder, with two carbon feet. The motors hang under the arm tips,
with the props below them.

Output: a folder with one STEP file per part and a top assembly file that
links to them (STEP AP214 external references). Parts used more than once
(motors, props, standoffs, screws) are one file with many instances. Open the
top file (super_drone.step) in a CAD program. Keep all files in one folder.
With --single-file, the same assembly tree goes into one STEP file instead, for
CAD programs that do not follow external references (Fusion 360).

Coordinates, all in mm: X forward, Y left, Z up. Z = 0 is the bottom face of
the main plate. The origin is the frame center.

Usage:
    uv run python super_drone.py
    uv run python super_drone.py --lidar-yaw 0 -o super_drone_b
    uv run python super_drone.py --no-guard  # without the LiDAR guard
    uv run python super_drone.py --single-file  # writes super_drone.step
"""

import argparse
import dataclasses
import math
import os
import shutil
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq
from OCP.gp import gp_Trsf
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorSurf, XCAFDoc_DocumentTool

from battery_holder import HolderParams, build_holder
from drone import make_esc
from drone_motor_full import FullMotorParams, build_motor
from fcu import PRESETS, FCParams, build_fc
from lidar_guard import RIB_ANGLES, GuardParams, build_guard, check_guard
from mid360s import Mid360Params, build_mid360s
from motor_prop import make_nut, nut_size, prop_z
from nuc13pro import NucParams, build_nuc
from nuc13pro import PCB_T as NUC_PCB_T
from prop import PropParams, build_prop, parse_spec

HERE = Path(__file__).resolve().parent
FRAME_DIR = HERE / "hardware" / "CarbonFiber"

# Plate files are drawn in their XZ plane with the thickness along +Y.
# In the drone: plate z -> X, plate x -> Y, plate y -> Z.
PLATE_ROT = ((0, 0, 1), (1, 0, 0), (0, 1, 0))
IDENTITY = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
MAIN_T, TOP_T, BAT_T = 5.0, 3.0, 2.0
PILLAR_TOP_H = 40.0  # main plate to top plate
PILLAR_BOT_H = 30.0  # main plate to battery plate
TOP_Z = MAIN_T + PILLAR_TOP_H  # bottom face of the top plate
BAT_Z = -PILLAR_BOT_H - BAT_T  # bottom face of the battery plate
PILLAR_TOP_XY = [(x, y) for x in (-57.5, 57.5) for y in (-57.5, 57.5)]
PILLAR_BOT_XY = [(x, y) for x in (-22.5, 22.5) for y in (-40.5, 40.5)]
NUC_HOLES_XY = [(x, y) for x in (-45.0, 45.0) for y in (-47.5, 47.5)]
LIDAR_HOLES = [(x, y) for x in (-24.0, 24.0) for y in (-18.0, 18.0)]  # Mid-360 M3 pattern
STACK_XY = [(x, y) for x in (-10.0, 10.0) for y in (-10.0, 10.0)]  # 20 x 20 FC stack
MOTOR_XY = 98.99  # motor centers at (+-MOTOR_XY, +-MOTOR_XY): 280 mm wheelbase
MOTOR_HOLE_R = 9.5  # 4 x M3 on a 19 mm circle, on the plate axes
# (name, sign x, sign y, prop direction seen from above), PX4 quad X order
MOTORS = [("fr", 1, -1, "ccw"), ("fl", 1, 1, "cw"), ("rl", -1, 1, "ccw"), ("rr", -1, -1, "cw")]
# The NUC sits turned 90 degrees: its back panel faces -X. It moves 0.45 mm along
# its own Y so its round holes and slots both fit the plate holes (90 mm apart).
NUC_SHIFT = 0.45
HOLDER = HolderParams()  # 3D printed battery holder (battery_holder.py); its flange sits under the battery plate
FOOT_Y = HOLDER.length / 2  # inner face of each foot (the holder end face)
M3_NUT_AF, M3_NUT_H = 5.5, 2.4  # ISO 4032 M3 nut, in the holder nut traps

CARBON = (0.13, 0.13, 0.14)
ALU = (0.72, 0.74, 0.78)
STEEL = (0.25, 0.25, 0.27)
NYLON = (0.90, 0.90, 0.88)
PRINT_BLACK = (0.10, 0.10, 0.11)
BATTERY_GRAY = (0.30, 0.31, 0.34)


@dataclass
class SuperParams:
    prop_spec: str = "7x4x3"  # HQProp 7x4x3 (inches: diameter x pitch x blades)
    nuc_standoff_h: float = 12.0  # standoffs under the NUC board
    lidar_yaw: int = 180  # 180 = the M12 connector faces the rear
    battery_l: float = 140.0  # DualSky XP33006HED 6S 3300 mAh
    battery_w: float = 44.0
    battery_h: float = 33.0
    guard: bool = True  # 3D printed LiDAR guard on the top plate (lidar_guard.py)
    hardware: bool = True  # screws, nuts and FC stack spacers

    def __post_init__(self):
        if self.lidar_yaw % 90:
            raise ValueError("lidar_yaw must be a multiple of 90 (the plate has holes for these)")
        for name in ("nuc_standoff_h", "battery_l", "battery_w", "battery_h"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        parse_spec(self.prop_spec)


# ---------------------------------------------------------------- placement


def loc(rot=IDENTITY, t=(0.0, 0.0, 0.0)) -> cq.Location:
    """Location from a rotation matrix (rows) and a translation."""
    tr = gp_Trsf()
    (a, b, c), (d, e, f), (g, h, i) = rot
    tr.SetValues(a, b, c, t[0], d, e, f, t[1], g, h, i, t[2])
    return cq.Location(tr)


def rot_z(deg: float) -> tuple:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return ((c, -s, 0), (s, c, 0), (0, 0, 1))


FLIP_X = ((1, 0, 0), (0, -1, 0), (0, 0, -1))  # 180 degrees about X: turns a part upside down


@dataclass
class Part:
    """One STEP file. make() returns the part in its own frame."""
    name: str
    make: object
    _built: cq.Assembly | None = None

    def build(self) -> cq.Assembly:
        if self._built is None:
            obj = self.make()
            if isinstance(obj, cq.Assembly):
                obj.name = self.name
                self._built = obj
            else:
                shape, color = obj
                self._built = cq.Assembly(shape, name=self.name, color=cq.Color(*color))
        return self._built


@dataclass
class Group:
    """A sub-assembly kept inside the top file."""
    name: str
    items: list  # (Part | Group, cq.Location)


# ---------------------------------------------------------------- simple parts


def cyl(d, z0, z1, x=0.0, y=0.0) -> cq.Solid:
    return cq.Solid.makeCylinder(d / 2, z1 - z0, cq.Vector(x, y, z0))


def tube(od, id_, h) -> cq.Shape:
    return cyl(od, 0, h).cut(cyl(id_, -1, h + 1)).clean()


def button_screw(d: float, length: float) -> cq.Shape:
    """Button head screw. The head is on z = 0..h, the shank points to -Z."""
    head_d, head_h = 1.9 * d, 0.55 * d
    head = cq.Workplane("XY").circle(head_d / 2).extrude(head_h).faces(">Z").edges().fillet(head_h * 0.6)
    socket = cq.Workplane("XY", origin=(0, 0, head_h * 0.4)).polygon(6, 0.6 * d / 0.866).extrude(head_h)
    shank = cyl(d - 0.1, -length, 0.01)  # 0.1 mm under the hole size
    return head.cut(socket).val().fuse(shank).clean()


def hex_nut(d: float, af: float, h: float) -> cq.Shape:
    return cq.Workplane("XY").polygon(6, af / 0.8660254).circle(d / 2).extrude(h).val()


def plate(file: str):
    return lambda: (cq.importers.importStep(str(FRAME_DIR / file)).val(), CARBON)


def make_battery(p: SuperParams) -> cq.Assembly:
    """6S LiPo pack: a rounded box with black end caps. Long side along X."""
    l, w, h = p.battery_l, p.battery_w, p.battery_h
    cap = 4.0
    body = cq.Workplane("XY").box(l - 2 * cap, w, h).edges("|X").fillet(3.0).val()
    caps = [cq.Workplane("XY").box(cap, w + 0.4, h + 0.4).edges("|X").fillet(3.2)
            .translate((sx * (l - cap) / 2, 0, 0)).val() for sx in (-1, 1)]
    assy = cq.Assembly(name="battery")
    assy.add(body, name="cells", color=cq.Color(*BATTERY_GRAY))
    assy.add(cq.Compound.makeCompound(caps), name="end_caps", color=cq.Color(*PRINT_BLACK))
    return assy


# ---------------------------------------------------------------- motors and props


def f90_params() -> FullMotorParams:
    """T-Motor F90 2806.5: 28 x 6.5 mm stator, 33.4 x 34.7 mm, 19 mm mount circle."""
    return FullMotorParams(
        stator_d=28.0, stator_h=6.5, stator_bore=16.0, magnet_t=1.3, bell_wall=1.0,
        base_d=30.0, base_h=5.0, clearance=1.5, mount_pcd=2 * MOTOR_HOLE_R,
        shaft_d=5.0, shaft_len=17.1, vent_pcd=22.0,
    )


def prop_params(p: SuperParams, direction: str) -> PropParams:
    values = parse_spec(p.prop_spec)
    values.update(hub_d=14.0, hub_h=8.5, root_chord=9.0, max_chord=17.0, tip_chord=6.0,
                  sweep=6.0, bore_d=5.0, direction=direction)
    return PropParams(**values)


def prop_loc(m: FullMotorParams, pp: PropParams) -> cq.Location:
    """The motor hangs upside down, so the prop is also turned over on the shaft.
    Its thrust then points up in the drone."""
    return loc(FLIP_X, (0, 0, prop_z(m) + pp.hub_h))


def check_props(m: FullMotorParams, pp: PropParams, prop: cq.Shape) -> None:
    _, nut_h = nut_size(m)
    if m.boss_h + pp.hub_h + nut_h > m.shaft_len:
        raise ValueError("the motor shaft is too short for the prop hub and nut")
    placed = prop.moved(prop_loc(m, pp))
    bell_zone = cyl(m.bell_d + 1.0, 0, m.bell_top_z + 0.3)
    if placed.intersect(bell_zone).Volume() > 1e-3:
        raise ValueError("prop blades hit the motor bell")
    lowest = -placed.BoundingBox().zmax  # the motor frame points down in the drone
    if lowest < -PILLAR_BOT_H + 1.0:
        raise ValueError(f"props reach z = {lowest:.1f}, below the battery plate top ({-PILLAR_BOT_H})")


# ---------------------------------------------------------------- assembly tree


def build_tree(p: SuperParams) -> Group:
    m = f90_params()
    props = {d: prop_params(p, d) for d in ("ccw", "cw")}
    prop_shapes = {d: build_prop(pp).val() for d, pp in props.items()}
    check_props(m, props["ccw"], prop_shapes["ccw"])

    nuc_params = NucParams()
    nuc_z = MAIN_T + p.nuc_standoff_h  # NUC PCB bottom face
    fc_p = FCParams(**PRESETS["20x20"])
    esc_p = dataclasses.replace(fc_p, board_w=36.0, board_l=32.0)  # outline is approximate

    # Parts (one STEP file each)
    main_plate = Part("main_plate_5mm", plate("5mm_main_board.STEP"))
    top_plate = Part("top_plate_3mm", plate("3mm_sec_board.step"))
    bat_plate = Part("battery_plate_2mm", plate("3mm_bat_board.step"))
    foot = Part("foot_4mm", plate("4mm-foot.STEP"))
    holder = Part("battery_holder", lambda: (build_holder(HOLDER), PRINT_BLACK))
    pillar_top = Part("pillar_m3_40mm", lambda: (tube(5.0, 3.0, PILLAR_TOP_H), ALU))
    pillar_bot = Part("pillar_m3_30mm", lambda: (tube(5.0, 3.0, PILLAR_BOT_H), ALU))
    nuc_standoff = Part(f"standoff_m3_{p.nuc_standoff_h:g}mm", lambda: (tube(5.0, 3.0, p.nuc_standoff_h), ALU))
    nuc = Part("nuc13pro", lambda: build_nuc(nuc_params))
    lidar = Part("mid360s", lambda: build_mid360s(Mid360Params()))
    fc = Part("flight_controller", lambda: build_fc(fc_p)[0])
    esc = Part("esc_4in1", lambda: make_esc(esc_p, 1))
    battery = Part("battery_6s_3300", lambda: make_battery(p))
    motor = Part("t_motor_f90", lambda: build_motor(m))
    prop_parts = {d: Part(f"prop_{p.prop_spec}_{d}", lambda d=d: (prop_shapes[d], (0.85, 0.85, 0.88)))
                  for d in props}
    prop_nut = Part("prop_nut_m5", lambda: (make_nut(m, 0.0).val(), (0.15, 0.35, 0.85)))

    # Frame
    frame = [(main_plate, loc(PLATE_ROT)),
             (top_plate, loc(PLATE_ROT, (0, 0, TOP_Z))),
             (bat_plate, loc(PLATE_ROT, (0, 0, BAT_Z)))]
    frame += [(pillar_top, loc(t=(x, y, MAIN_T))) for x, y in PILLAR_TOP_XY]
    frame += [(pillar_bot, loc(t=(x, y, -PILLAR_BOT_H))) for x, y in PILLAR_BOT_XY]
    # The holder flange is under the battery plate. Holder X -> drone Y, holder Z -> drone -Z.
    frame.append((holder, loc(((0, 1, 0), (1, 0, 0), (0, 0, -1)), (0, 0, BAT_Z))))
    # Feet: on the holder end faces. Foot x -> drone X, foot y (up) -> drone Z.
    # The foot file has its top edge at y = 4 and its screw holes at y = -1. The holes
    # must be 5 mm under the flange face, so the top edge meets the battery plate.
    foot_z = BAT_Z - 4.0
    frame.append((foot, loc(((1, 0, 0), (0, 0, -1), (0, 1, 0)), (0, -FOOT_Y, foot_z))))
    frame.append((foot, loc(((-1, 0, 0), (0, 0, 1), (0, 1, 0)), (0, FOOT_Y, foot_z))))

    # Compute: NUC on standoffs, back panel to the rear
    compute = [(nuc_standoff, loc(t=(x, y, MAIN_T))) for x, y in NUC_HOLES_XY]
    compute.append((nuc, loc(rot_z(90), (-NUC_SHIFT, 0, nuc_z))))

    # Flight stack under the main plate: FC on top, ESC below it
    fc_z, esc_z = -8.0, -18.0
    g = fc_p.grommet_h + 0.035  # the grommet flange sits on a 0.035 mm copper pad
    stack = [(fc, loc(t=(0, 0, fc_z))), (esc, loc(t=(0, 0, esc_z)))]

    # Sensing: Mid-360S on the top plate
    sensing = [(lidar, loc(rot_z(p.lidar_yaw), (0, 0, TOP_Z + TOP_T)))]
    guard_p = GuardParams()
    if p.guard:
        guard_shape = build_guard(guard_p)
        check_guard(guard_shape)
        guard = Part("lidar_guard", lambda: (guard_shape, PRINT_BLACK))
        sensing.append((guard, loc(rot_z(p.lidar_yaw), (0, 0, TOP_Z + TOP_T))))

    # Power: battery under the holder
    holder_bottom = BAT_Z - HOLDER.height
    # Long side along Y, the same as the holder
    power = [(battery, loc(rot_z(90), (0, 0, holder_bottom - p.battery_h / 2)))]

    # Propulsion: each motor hangs under an arm tip
    propulsion = []
    for name, sx, sy, direction in MOTORS:
        unit = Group(f"motor_{name}", [
            (motor, loc()),
            (prop_parts[direction], prop_loc(m, props[direction])),
            (prop_nut, loc(t=(0, 0, prop_z(m) + props[direction].hub_h))),
        ])
        propulsion.append((unit, loc(FLIP_X, (sx * MOTOR_XY, sy * MOTOR_XY, 0))))

    groups = [Group("frame", frame), Group("compute", compute), Group("flight_stack", stack),
              Group("sensing", sensing), Group("power", power), Group("propulsion", propulsion)]

    if p.hardware:
        m3x6 = Part("screw_m3x6_button", lambda: (button_screw(3.0, 6.0), STEEL))
        m3x8 = Part("screw_m3x8_button", lambda: (button_screw(3.0, 8.0), STEEL))
        m3x10 = Part("screw_m3x10_button", lambda: (button_screw(3.0, 10.0), STEEL))
        up = IDENTITY  # head up, shank down
        hw = [(m3x6, loc(up, (x, y, TOP_Z + TOP_T))) for x, y in PILLAR_TOP_XY]
        hw += [(m3x6, loc(up, (x, y, nuc_z + NUC_PCB_T))) for x, y in NUC_HOLES_XY]
        # Lidar screws: from under the top plate into the sensor base.
        hw += [(m3x6, loc(FLIP_X, (x, y, TOP_Z)))
               for x, y in (rot_xy(h, p.lidar_yaw) for h in LIDAR_HOLES)]
        # Battery screws: through the holder flange and battery plate into the pillars.
        hw += [(m3x10, loc(FLIP_X, (x, y, BAT_Z - HOLDER.flange_t))) for x, y in PILLAR_BOT_XY]
        # Motor screws: through the main plate into the motor base.
        for _, sx, sy, _ in MOTORS:
            cx, cy = sx * MOTOR_XY, sy * MOTOR_XY
            for dx, dy in ((MOTOR_HOLE_R, 0), (-MOTOR_HOLE_R, 0), (0, MOTOR_HOLE_R), (0, -MOTOR_HOLE_R)):
                hw.append((m3x8, loc(up, (cx + dx, cy + dy, MAIN_T))))
        # Foot screws: through each foot into the holder end blocks, heads outside,
        # into M3 nuts in the holder nut traps. The nut sits at the outer end of its trap.
        m3x12 = Part("screw_m3x12_button", lambda: (button_screw(3.0, 12.0), STEEL))
        m3_nut = Part("nut_m3", lambda: (hex_nut(3.0, M3_NUT_AF, M3_NUT_H), STEEL))
        nut_axis_y = ((0, 1, 0), (0, 0, 1), (1, 0, 0))  # nut axis along drone +Y, hex corners up and down
        trap_end = FOOT_Y - HOLDER.end_block_l + HOLDER.nut_depth  # outer end of each nut trap
        foot_z = BAT_Z - HOLDER.foot_hole_z
        for sy in (-1, 1):
            # The screw's local +Z (head side) points away from the holder.
            side = ((1, 0, 0), (0, 0, -1), (0, 1, 0)) if sy < 0 else ((1, 0, 0), (0, 0, 1), (0, -1, 0))
            nut_y = trap_end - M3_NUT_H if sy > 0 else -trap_end
            for x in (-HOLDER.foot_hole_y, HOLDER.foot_hole_y):
                hw.append((m3x12, loc(side, (x, sy * (FOOT_Y + 4.0), foot_z))))
                hw.append((m3_nut, loc(nut_axis_y, (x, nut_y, foot_z))))
        # Guard screws: from under the top plate into the guard feet, into M3 nuts in the foot traps.
        # The nut flats run along the rib, the same as the trap.
        if p.guard:
            for a in RIB_ANGLES:
                a += p.lidar_yaw
                x, y = rot_xy((guard_p.rib_r, 0.0), a)
                hw.append((m3x10, loc(FLIP_X, (x, y, TOP_Z))))
                hw.append((m3_nut, loc(rot_z(a), (x, y, TOP_Z + TOP_T + guard_p.nut_z))))
        groups.append(Group("hardware", hw))

        # FC stack: M2 screws from the top of the main plate, spacers between the boards, nuts under the ESC.
        top_gap = -(fc_z + fc_p.board_t + g)  # main plate bottom to FC top grommet
        mid_gap = (fc_z - g) - (esc_z + esc_p.board_t + g)  # FC bottom grommet to ESC top grommet
        nut_z = esc_z - g
        screw_len = MAIN_T - (nut_z - 2.0)
        spacer_top = Part(f"spacer_m2_{top_gap:.1f}mm", lambda: (tube(4.0, 2.2, top_gap), NYLON))
        spacer_mid = Part(f"spacer_m2_{mid_gap:.1f}mm", lambda: (tube(4.0, 2.2, mid_gap), NYLON))
        m2 = Part(f"screw_m2x{screw_len:.0f}_button", lambda: (button_screw(2.0, screw_len), STEEL))
        m2_nut = Part("nut_m2", lambda: (hex_nut(2.0, 4.0, 1.6), STEEL))
        for x, y in STACK_XY:
            stack += [(spacer_top, loc(t=(x, y, -top_gap))),
                      (spacer_mid, loc(t=(x, y, esc_z + esc_p.board_t + g))),
                      (m2, loc(t=(x, y, MAIN_T))),
                      (m2_nut, loc(t=(x, y, nut_z - 1.6)))]
    return Group("super_drone", groups_as_items(groups))


def rot_xy(pt, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return (round(c * pt[0] - s * pt[1], 6), round(s * pt[0] + c * pt[1], 6))


def groups_as_items(groups):
    return [(g, loc()) for g in groups]


# ---------------------------------------------------------------- STEP export


def collect_parts(node, out: dict) -> None:
    for item, _ in node.items:
        if isinstance(item, Group):
            collect_parts(item, out)
        else:
            out.setdefault(item.name, item)


def new_doc():
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    XCAFApp_Application.GetApplication_s().InitDocument(doc)
    return doc, XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())


def set_name(label, name):
    TDataStd_Name.Set_s(label, TCollection_ExtendedString(name))


def add_shape(doc, tool, assy: cq.Assembly, name: str):
    """Add the shape of an assembly node (no children) as one named, colored product."""
    shape = assy.obj if isinstance(assy.obj, cq.Shape) else cq.Compound.makeCompound(assy.shapes)
    label = tool.AddShape(shape.wrapped, False)
    set_name(label, name)
    if assy.color is not None:
        XCAFDoc_DocumentTool.ColorTool_s(doc.Main()).SetColor(label, assy.color.wrapped, XCAFDoc_ColorSurf)
    return label


def add_part(doc, tool, assy: cq.Assembly):
    """Add a built part: one product, or a sub-assembly of its colored pieces."""
    if not assy.children:
        return add_shape(doc, tool, assy, assy.name)
    label = tool.NewShape()
    set_name(label, assy.name)
    if assy.obj is not None:
        tool.AddComponent(label, add_shape(doc, tool, assy, f"{assy.name}_part"), cq.Location().wrapped)
    for child in assy.children:
        tool.AddComponent(label, add_part(doc, tool, child), child.loc.wrapped)
    return label


def add_tree(tool, group: Group, labels: dict):
    """Add the group tree. Every use of a part is an instance of its one label."""
    label = tool.NewShape()
    set_name(label, group.name)
    for item, where in group.items:
        child = add_tree(tool, item, labels) if isinstance(item, Group) else labels[item.name]
        tool.AddComponent(label, child, where.wrapped)
    return label


def step_writer(assembly: bool) -> STEPCAFControl_Writer:
    writer = STEPCAFControl_Writer()
    writer.SetNameMode(True)
    writer.SetColorMode(True)
    Interface_Static.SetIVal_s("write.step.assembly", int(assembly))  # 0 keeps a compound as one product
    return writer


def export_part(part: Part, path: str) -> None:
    """Write one part file, with the part as the root product and every piece named.
    cq.Assembly.export wraps a single shape in an unnamed root and leaves some
    pieces unnamed, and the CAD tree then shows them twice (instance, then product)."""
    assy = part.build()
    doc, tool = new_doc()
    add_part(doc, tool, assy)
    tool.UpdateAssemblies()
    writer = step_writer(assembly=bool(assy.children))
    if not writer.Transfer(doc, STEPControl_AsIs) or writer.Write(path) != IFSelect_RetDone:
        raise RuntimeError(f"STEP export failed for {part.name}")


def write_single(root: Group, path: Path) -> list[str]:
    """Write the whole assembly into one STEP file, for CAD programs that do not
    follow external references (Fusion 360). The tree and part instances are kept."""
    parts: dict[str, Part] = {}
    collect_parts(root, parts)
    doc, tool = new_doc()
    labels = {name: add_part(doc, tool, part.build()) for name, part in parts.items()}
    add_tree(tool, root, labels)
    tool.UpdateAssemblies()
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = step_writer(assembly=True)
    if not writer.Transfer(doc, STEPControl_AsIs) or writer.Write(str(path)) != IFSelect_RetDone:
        raise RuntimeError("STEP export failed")
    return sorted(parts)


def write_multifile(root: Group, out_dir: Path) -> list[str]:
    """Write the top assembly with external references, then one STEP file per part."""
    parts: dict[str, Part] = {}
    collect_parts(root, parts)
    for part in parts.values():
        part.build()

    doc, tool = new_doc()

    # Each part gets a placeholder (its bounding box). Its file is replaced below.
    labels = {}
    for name, part in parts.items():
        bb = part.build().toCompound().BoundingBox()
        placeholder = cq.Solid.makeBox(max(bb.xlen, 0.1), max(bb.ylen, 0.1), max(bb.zlen, 0.1),
                                       cq.Vector(bb.xmin, bb.ymin, bb.zmin))
        labels[name] = tool.AddShape(placeholder.wrapped, False)
        set_name(labels[name], name)

    add_tree(tool, root, labels)
    tool.UpdateAssemblies()

    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)
    cwd = os.getcwd()
    os.chdir(out_dir)  # the writer puts the linked files in the current folder
    try:
        writer = step_writer(assembly=True)
        top = f"{root.name}.step"
        if not writer.Transfer(doc, STEPControl_AsIs, ""):
            raise RuntimeError("STEP transfer failed")
        writer.Write(top)
        # Replace each placeholder with the real part, and use the .step extension.
        text = Path(top).read_text()
        for name, part in parts.items():
            Path(f"{name}.stp").unlink(missing_ok=True)
            export_part(part, f"{name}.step")
            text = text.replace(f"'{name}.stp", f"'{name}.step")
        Path(top).write_text(text)
    finally:
        os.chdir(cwd)
    return sorted(parts)


# ---------------------------------------------------------------- CLI


def main() -> None:
    parser = argparse.ArgumentParser(description="Build the SUPER quadrotor as a STEP assembly.")
    for f in dataclasses.fields(SuperParams):
        flag = "--" + f.name.replace("_", "-")
        if f.type is bool:
            parser.add_argument(flag, dest=f.name, action=argparse.BooleanOptionalAction, default=None,
                                help=f"default: {f.default}")
        else:
            parser.add_argument(flag, dest=f.name, type=f.type, default=None, help=f"default: {f.default}")
    parser.add_argument("-o", "--output", default="super_drone",
                        help="output folder (or file name without .step, with --single-file)")
    parser.add_argument("--single-file", action="store_true",
                        help="write one self-contained STEP file (for Fusion 360) instead of a folder")
    args = vars(parser.parse_args())
    out, single = Path(args.pop("output")), args.pop("single_file")
    try:
        p = SuperParams(**{k: v for k, v in args.items() if v is not None})
        root = build_tree(p)
    except ValueError as e:
        parser.error(str(e))
    if single:
        path = out.with_suffix(".step")
        names = write_single(root, path)
        print(f"Wrote {path} with {len(names)} parts")
        return
    names = write_multifile(root, out)
    print(f"Wrote {out}/{root.name}.step and {len(names)} part files:")
    for n in names:
        print(f"  {n}.step")


if __name__ == "__main__":
    main()
