"""Parametric fake flight controller (FC) model, exported as a colored STEP assembly.

The board looks like a typical drone FC: MCU in the middle, IMU under it, USB-C and
JST-SH connectors on the edges, soft-mount grommets, and small chips and passives on
both sides. The parts are placeholders for fit checks, not a real circuit. A seed
controls the random placement of the small parts, so the same seed gives the same board.

The board center is at X = Y = 0. The board bottom face is at Z = 0.
All units are mm.

Usage:
    uv run python fcu.py                                 # 30.5 x 30.5 mm mount pattern
    uv run python fcu.py --preset 20x20 -o fc_20x20.step
    uv run python fcu.py --usb-edge +x --connectors 8,6 --seed 7 --mask green
"""

import argparse
import dataclasses
import math
import random
from collections import Counter, defaultdict
from dataclasses import dataclass, field

import cadquery as cq

ORIGIN = cq.Vector(0, 0, 0)
X_AXIS = cq.Vector(1, 0, 0)
Z_AXIS = cq.Vector(0, 0, 1)

EDGES = {"+x": 0, "+y": 90, "-x": 180, "-y": 270}  # edge name -> rotation of an edge part
MASK_COLORS = {
    "black": (0.05, 0.05, 0.06),
    "green": (0.05, 0.35, 0.12),
    "blue": (0.05, 0.15, 0.45),
    "red": (0.55, 0.05, 0.05),
    "purple": (0.30, 0.08, 0.40),
}
GROUP_COLORS = {
    "copper": (0.85, 0.65, 0.20),
    "grommets": (0.20, 0.20, 0.22),
    "ics": (0.08, 0.08, 0.08),
    "pins": (0.80, 0.80, 0.82),
    "caps": (0.72, 0.58, 0.40),
    "resistors": (0.10, 0.10, 0.10),
    "terminals": (0.85, 0.85, 0.88),
    "metal": (0.75, 0.75, 0.78),
    "inductors": (0.25, 0.25, 0.27),
    "connectors": (0.93, 0.90, 0.80),
    "leds": (0.95, 0.95, 1.00),
    "usb_tongue": (0.05, 0.05, 0.05),
}
# Chip sizes (length, width, height) and the chance to pick each one.
PASSIVES = [((1.0, 0.5, 0.35), 0.6), ((1.6, 0.8, 0.45), 0.3), ((2.0, 1.25, 0.6), 0.1)]

PRESETS = {
    "30x30": {},
    "20x20": dict(
        board_w=25.5, board_l=25.5, corner_r=2.5, mount_pattern=20.0, hole_d=3.0,
        grommet_id=2.0, grommet_od=4.5, mcu_body=7.0, mcu_pins=12, connectors="8,4",
        passives_top=15, passives_bottom=25,
    ),
}


@dataclass
class FCParams:
    # Board
    board_w: float = 36.0  # size in X
    board_l: float = 36.0  # size in Y
    board_t: float = 1.6  # PCB thickness
    corner_r: float = 3.0
    mask: str = field(default="black", metadata={"choices": tuple(MASK_COLORS)})
    # Mounting
    mount_pattern: float = 30.5  # hole spacing (square pattern)
    hole_d: float = 4.0  # board hole diameter
    grommets: bool = True  # soft-mount rubber grommets in the holes
    grommet_id: float = 3.0  # grommet inner diameter (screw size)
    grommet_od: float = 6.0  # grommet flange diameter
    grommet_h: float = 1.0  # grommet flange height on each side
    # MCU (LQFP, on the top side in the middle)
    mcu_body: float = 10.0  # body size (LQFP-64: 10, LQFP-100: 14)
    mcu_pins: int = 16  # pins per side
    mcu_pitch: float = 0.5
    # Edge parts
    usb_edge: str = field(default="-y", metadata={"choices": tuple(EDGES)})
    connectors: str = "8,6,4,4"  # JST-SH pin counts, spread over the other edges
    # Small parts
    passives_top: int = 30
    passives_bottom: int = 50
    seed: int = 1

    @property
    def keepout_r(self) -> float:
        """Radius around each mount hole with no parts."""
        ring = self.hole_d + 2.0
        return max(ring, self.grommet_od if self.grommets else 0.0) / 2 + 0.3

    @property
    def hole_points(self) -> list[tuple[float, float]]:
        h = self.mount_pattern / 2
        return [(h, h), (-h, h), (-h, -h), (h, -h)]

    @property
    def connector_pins(self) -> list[int]:
        return [int(s) for s in self.connectors.split(",") if s.strip()]

    def __post_init__(self):
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if f.type is float and value <= 0:
                raise ValueError(f"{f.name} must be positive, got {value}")
        if self.mask not in MASK_COLORS:
            raise ValueError(f"mask must be one of {', '.join(MASK_COLORS)}")
        if self.usb_edge not in EDGES:
            raise ValueError(f"usb_edge must be one of {', '.join(EDGES)}")
        try:
            pins = self.connector_pins
        except ValueError:
            raise ValueError("connectors must be a comma list of pin counts, e.g. 8,6,4")
        if any(not 2 <= n <= 12 for n in pins):
            raise ValueError("each connector must have 2 to 12 pins")
        if min(self.passives_top, self.passives_bottom, self.mcu_pins) < 0:
            raise ValueError("counts must not be negative")
        if self.mcu_pins < 1 or self.mcu_pins * self.mcu_pitch >= self.mcu_body - 0.5:
            raise ValueError("mcu_pins * mcu_pitch must be smaller than mcu_body")
        if self.corner_r * 2 >= min(self.board_w, self.board_l):
            raise ValueError("corner_r is too large for the board")
        if self.mount_pattern / 2 + self.hole_d / 2 + 0.5 > min(self.board_w, self.board_l) / 2:
            raise ValueError("mount holes are outside the board")
        if self.grommets and not self.grommet_id < self.hole_d < self.grommet_od:
            raise ValueError("grommets need grommet_id < hole_d < grommet_od")


# Part builders. Each returns (shapes, footprint_x, footprint_y).
# Shapes are centered at X = Y = 0 and stand on Z = 0. Edge parts face +X.

Shapes = list[tuple[str, cq.Shape]]


def box(x0: float, y0: float, z0: float, dx: float, dy: float, dz: float) -> cq.Solid:
    return cq.Solid.makeBox(dx, dy, dz, cq.Vector(x0, y0, z0))


def cbox(dx: float, dy: float, dz: float, z0: float = 0.0, cx: float = 0.0, cy: float = 0.0) -> cq.Solid:
    """Box centered in X and Y."""
    return box(cx - dx / 2, cy - dy / 2, z0, dx, dy, dz)


def ic(body_x: float, body_y: float, h: float, pins: int, pitch: float,
       four_sides: bool, pin_len: float = 1.0) -> tuple[Shapes, float, float]:
    """Gull-wing IC. Pins are on the +-Y sides, and also on the +-X sides if four_sides."""
    shapes: Shapes = [("ics", cbox(body_x, body_y, h - 0.1, z0=0.1))]
    pin_w = min(0.45 * pitch, 0.45)
    offsets = [(i - (pins - 1) / 2) * pitch for i in range(pins)]
    for sign in (1, -1):
        for t in offsets:
            y0 = body_y / 2 if sign > 0 else -body_y / 2 - pin_len
            shapes.append(("pins", box(t - pin_w / 2, y0, 0, pin_w, pin_len, 0.15)))
            if four_sides:
                x0 = body_x / 2 if sign > 0 else -body_x / 2 - pin_len
                shapes.append(("pins", box(x0, t - pin_w / 2, 0, pin_len, pin_w, 0.15)))
    fx = body_x + (2 * pin_len if four_sides else 0.0)
    return shapes, fx, body_y + 2 * pin_len


def block(group: str, dx: float, dy: float, h: float) -> tuple[Shapes, float, float]:
    """Plain package with no visible pins (QFN, crystal, inductor)."""
    return [(group, cbox(dx, dy, h))], dx, dy


def chip(group: str, length: float, width: float, h: float) -> tuple[Shapes, float, float]:
    """Two-terminal chip part (resistor, capacitor, LED) with metal ends."""
    end = 0.2 * length
    shapes: Shapes = [
        (group, cbox(length - 2 * end, width, h)),
        ("terminals", cbox(end, width, h, cx=(length - end) / 2)),
        ("terminals", cbox(end, width, h, cx=-(length - end) / 2)),
    ]
    return shapes, length, width


def button() -> tuple[Shapes, float, float]:
    return [("metal", cbox(3.0, 2.0, 0.8)), ("ics", cbox(1.0, 0.8, 0.6, z0=0.8))], 3.0, 2.0


def usb_c() -> tuple[Shapes, float, float]:
    depth, width, h = 7.35, 8.94, 3.26
    open_depth = 6.0
    shell = (
        cq.Workplane("YZ", origin=(-depth / 2, 0, 0)).center(0, h / 2)
        .slot2D(width, h).extrude(depth)
    )
    opening = (
        cq.Workplane("YZ", origin=(depth / 2 - open_depth, 0, 0)).center(0, h / 2)
        .slot2D(width - 0.6, h - 0.6).extrude(open_depth + 0.1)
    )
    tongue = box(depth / 2 - open_depth, -3.3, h / 2 - 0.35, open_depth - 1.0, 6.6, 0.7)
    return [("metal", shell.cut(opening).val()), ("usb_tongue", tongue)], depth, width


def jst_sh(pins: int) -> tuple[Shapes, float, float]:
    """JST-SH side-entry connector, 1 mm pitch."""
    depth, width, h = 4.25, pins + 3.0, 2.95
    body = cbox(depth, width, h)
    pocket = box(depth / 2 - 3.0, -(pins + 0.6) / 2, 0.4, 3.1, pins + 0.6, 2.0)
    return [("connectors", body.cut(pocket))], depth, width


def ring(od: float, id_: float, z0: float, h: float) -> cq.Shape:
    return cq.Workplane("XY").workplane(offset=z0).circle(od / 2).circle(id_ / 2).extrude(h).val()


# Layout


class Layout:
    """Places parts on the board and keeps a list of used rectangles on each side."""

    margin = 0.25  # space between parts
    edge = 0.6  # space between parts and the board edge

    def __init__(self, p: FCParams):
        self.p = p
        self.used: dict[str, list[tuple[float, float, float, float]]] = {"top": [], "bottom": []}
        self.groups: dict[str, list[cq.Shape]] = defaultdict(list)
        self.counts: Counter = Counter()
        self.skipped: list[str] = []

    def problem(self, side: str, rect, on_edge: bool) -> str | None:
        x0, y0, x1, y1 = rect
        w, l = self.p.board_w / 2 - self.edge, self.p.board_l / 2 - self.edge
        if not on_edge and (x0 < -w or x1 > w or y0 < -l or y1 > l):
            return "is outside the board"
        for hx, hy in self.p.hole_points:
            dx = max(x0 - hx, 0, hx - x1)
            dy = max(y0 - hy, 0, hy - y1)
            if math.hypot(dx, dy) < self.p.keepout_r:
                return "hits a mount hole"
        m = self.margin
        for u0, v0, u1, v1 in self.used[side]:
            if x0 < u1 + m and u0 < x1 + m and y0 < v1 + m and v0 < y1 + m:
                return "hits another part"
        return None

    def place(self, name: str, side: str, part, cx: float, cy: float, rot: int = 0,
              on_edge: bool = False, required: bool = True) -> bool:
        shapes, fx, fy = part
        if rot % 180:
            fx, fy = fy, fx
        rect = (cx - fx / 2, cy - fy / 2, cx + fx / 2, cy + fy / 2)
        problem = self.problem(side, rect, on_edge)
        if problem:
            if required:
                raise ValueError(f"{name} {problem}: change the board size or the part settings")
            return False
        z = self.p.board_t if side == "top" else 0.0
        for group, s in shapes:
            if side == "bottom":
                s = s.rotate(ORIGIN, X_AXIS, 180)
            self.groups[group].append(s.rotate(ORIGIN, Z_AXIS, rot).translate((cx, cy, z)))
        self.used[side].append(rect)
        self.counts[name] += 1
        return True

    def place_on_edge(self, name: str, edge: str, part, along: float, overhang: float = 0.0):
        """Put an edge part on the top side, facing out of the given edge."""
        _, depth, _ = part
        rot = EDGES[edge]
        half = (self.p.board_w if rot % 180 == 0 else self.p.board_l) / 2
        x, y = half + overhang - depth / 2, along
        a = math.radians(rot)
        cx, cy = x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)
        self.place(name, "top", part, round(cx, 6), round(cy, 6), rot, on_edge=True)

    def place_random(self, rng: random.Random, name: str, side: str, part,
                     required: bool = True, tries: int = 400) -> None:
        _, fx, fy = part
        for _ in range(tries):
            rot = rng.choice((0, 90))
            hx = (fy if rot else fx) / 2 + self.edge
            hy = (fx if rot else fy) / 2 + self.edge
            cx = rng.uniform(-self.p.board_w / 2 + hx, self.p.board_w / 2 - hx)
            cy = rng.uniform(-self.p.board_l / 2 + hy, self.p.board_l / 2 - hy)
            if self.place(name, side, part, cx, cy, rot, required=False):
                return
        self.skipped.append(f"{name} ({side})")


def make_board(p: FCParams) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(p.board_w, p.board_l, p.board_t, centered=(True, True, False))
        .edges("|Z")
        .fillet(p.corner_r)
        .faces(">Z")
        .workplane()
        .pushPoints(p.hole_points)
        .hole(p.hole_d)
    )


def mount_hardware(p: FCParams, lay: Layout) -> None:
    pad_od, cu = p.hole_d + 2.0, 0.035
    for hx, hy in p.hole_points:
        for z0 in (p.board_t, -cu):
            lay.groups["copper"].append(ring(pad_od, p.hole_d, z0, cu).translate((hx, hy, 0)))
        if p.grommets:
            g = [
                ring(p.hole_d, p.grommet_id, 0, p.board_t),
                ring(p.grommet_od, p.grommet_id, p.board_t + cu, p.grommet_h),
                ring(p.grommet_od, p.grommet_id, -cu - p.grommet_h, p.grommet_h),
            ]
            lay.groups["grommets"].append(g[0].fuse(g[1], g[2]).clean().translate((hx, hy, 0)))


def build_fc(p: FCParams) -> tuple[cq.Assembly, Layout]:
    rng = random.Random(p.seed)
    lay = Layout(p)
    mount_hardware(p, lay)

    # Fixed parts: MCU on top and IMU under it, USB-C and connectors on the edges.
    lay.place("mcu", "top", ic(p.mcu_body, p.mcu_body, 1.4, p.mcu_pins, p.mcu_pitch, True), 0, 0)
    lay.place("imu", "bottom", block("ics", 3.0, 3.0, 0.9), 0, 0)
    lay.place_on_edge("usb_c", p.usb_edge, usb_c(), 0, overhang=0.6)

    order = list(EDGES)
    i = order.index(p.usb_edge)
    other_edges = order[i + 1:] + order[:i]
    per_edge: dict[str, list[int]] = defaultdict(list)
    for k, pins in enumerate(p.connector_pins):
        per_edge[other_edges[k % 3]].append(pins)
    gap = 1.5
    for edge, pin_list in per_edge.items():
        parts = [jst_sh(n) for n in pin_list]
        total = sum(w for _, _, w in parts) + gap * (len(parts) - 1)
        along = -total / 2
        for part, n in zip(parts, pin_list):
            w = part[2]
            lay.place_on_edge(f"jst_sh_{n}pin", edge, part, along + w / 2)
            along += w + gap

    # Other chips at random free places.
    misc = [
        ("osd", "top", ic(9.7, 4.4, 1.2, 14, 0.65, False, pin_len=0.6)),
        ("crystal", "top", block("metal", 3.2, 2.5, 0.8)),
        ("inductor", "top", block("inductors", 4.0, 4.0, 2.0)),
        ("button", "top", button()),
        ("led", "top", chip("leds", 1.6, 0.8, 0.6)),
        ("led", "top", chip("leds", 1.6, 0.8, 0.6)),
        ("led", "top", chip("leds", 1.6, 0.8, 0.6)),
        ("flash", "bottom", ic(4.9, 3.9, 1.5, 4, 1.27, False)),
        ("baro", "bottom", block("ics", 2.0, 2.5, 0.8)),
        ("regulator", "bottom", ic(2.9, 1.6, 1.1, 3, 0.95, False, pin_len=0.6)),
        ("regulator", "bottom", ic(2.9, 1.6, 1.1, 3, 0.95, False, pin_len=0.6)),
        ("inductor", "bottom", block("inductors", 4.0, 4.0, 2.0)),
    ]
    for name, side, part in misc:
        lay.place_random(rng, name, side, part)

    sizes, weights = zip(*PASSIVES)
    for side, count in (("top", p.passives_top), ("bottom", p.passives_bottom)):
        for _ in range(count):
            size = rng.choices(sizes, weights)[0]
            kind = rng.choice(("caps", "resistors"))
            lay.place_random(rng, "passive", side, chip(kind, *size), tries=150)

    return to_assembly(p, lay, "flight_controller"), lay


def to_assembly(p: FCParams, lay: Layout, name: str) -> cq.Assembly:
    """Board plus one colored compound per part group."""
    assy = cq.Assembly(name=name)
    assy.add(make_board(p), name="pcb", color=cq.Color(*MASK_COLORS[p.mask]))
    for group, color in GROUP_COLORS.items():
        if lay.groups[group]:
            assy.add(cq.Compound.makeCompound(lay.groups[group]), name=group, color=cq.Color(*color))
    return assy


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a fake flight controller STEP model.")
    parser.add_argument("--preset", choices=tuple(PRESETS), help="board size preset. Other flags override it.")
    for f in dataclasses.fields(FCParams):
        flag = "--" + f.name.replace("_", "-")
        # Default None marks flags the user did not give.
        if f.type is bool:
            parser.add_argument(flag, action=argparse.BooleanOptionalAction, default=None,
                                help=f"default: {f.default}")
        else:
            parser.add_argument(flag, type=f.type, default=None, choices=f.metadata.get("choices"),
                                help=f"default: {f.default}")
    parser.add_argument("-o", "--output", default="fcu.step", help="STEP output path")
    args = vars(parser.parse_args())
    output = args.pop("output")
    preset = args.pop("preset")

    try:
        values = dict(PRESETS[preset]) if preset else {}
        values.update({k: v for k, v in args.items() if v is not None})
        params = FCParams(**values)
        assy, lay = build_fc(params)
    except ValueError as e:
        parser.error(str(e))

    assy.export(output)
    bb = assy.toCompound().BoundingBox()
    print(f"Wrote {output}")
    print("Parts: " + ", ".join(f"{n} x{c}" for n, c in lay.counts.items()))
    if lay.skipped:
        print(f"No free space for {len(lay.skipped)} part(s): " + ", ".join(sorted(set(lay.skipped))))
    print(f"Size: X {bb.xlen:.2f}  Y {bb.ylen:.2f}  Z {bb.zlen:.2f} mm")


if __name__ == "__main__":
    main()
