"""Intel NUC 13 Pro board (NUC13ANB) without its case, as a STEP assembly.

The model has the board, the ports, the memory, the M.2 cards and the CPU
cooler (copper plate, heat pipes, fin stack and blower fan). Sizes and
positions come from the ASUS NUC 13 Pro technical product specification and
the official ASUS mechanical STEP file. Small parts (passives, pins, labels,
cables) are left out. Each part is a simple shape with the real outer size.

Coordinates, all in mm. The origin is the board center.
    X: across the board.
    Y: +Y is the back panel (HDMI, LAN, Thunderbolt, DC in).
       -Y is the front panel (USB-A, audio, power button).
    Z: Z = 0 is the PCB face with the ports, memory and M.2 slots. These parts
       hang toward -Z. The CPU and the cooler are on the other face, toward +Z.

Usage:
    uv run python nuc13pro.py
    uv run python nuc13pro.py --cooler heatsink --ram-sticks 1 -o nuc_1dimm.step
    uv run python nuc13pro.py --cooler none --ssd-2242 --no-ssd-2280 -o nuc_board.step
"""

import argparse
import dataclasses
import math
from dataclasses import dataclass, field

import cadquery as cq

PCB_T = 1.56
PCB_COLORS = {
    "green": (0.05, 0.32, 0.18),
    "black": (0.07, 0.07, 0.08),
    "blue": (0.05, 0.15, 0.40),
}
METAL = (0.78, 0.78, 0.80)
COPPER = (0.85, 0.50, 0.30)
BLACK = (0.08, 0.08, 0.09)
DARK = (0.18, 0.18, 0.20)
IVORY = (0.92, 0.90, 0.84)
GOLD = (0.85, 0.70, 0.25)
USB3_BLUE = (0.10, 0.30, 0.85)
DIMM_GREEN = (0.10, 0.45, 0.20)
LED_WHITE = (0.95, 0.95, 0.98)
DIE_GRAY = (0.45, 0.47, 0.50)

# Board outline as (x, y, corner radius), counter-clockwise from the front-left corner.
PCB_OUTLINE = [
    (-50.8, -49.25, 3.0), (-27.0, -49.25, 1.0), (-27.0, -51.7, 1.0),
    (12.0, -51.7, 0.0), (12.0, -52.05, 0.0), (24.45, -52.05, 1.5),
    (24.45, -49.68, 0.5), (25.95, -49.68, 0.5), (25.95, -50.38, 0.0),
    (50.8, -50.38, 3.0), (50.8, 52.05, 4.5),
    # LAN jack notch
    (12.1, 52.05, 0.5), (12.1, 40.95, 0.5), (-4.48, 40.95, 0.5), (-4.48, 52.05, 0.5),
    # Dual USB-A notch
    (-6.33, 52.05, 0.5), (-6.33, 44.85, 0.3), (-7.65, 44.85, 0.3), (-7.65, 37.02, 0.5),
    (-21.15, 37.02, 0.5), (-21.15, 44.85, 0.3), (-22.48, 44.85, 0.3), (-22.48, 52.05, 0.5),
    (-50.8, 52.05, 4.5),
]
SIDE_NOTCH_Y = 36.75  # half-round notch (r = 2) on both side edges
HEATSINK_HOLES = [(-21.05, -28.12), (18.95, -28.12), (-21.05, 21.88), (18.95, 21.88)]
M2_2280_STANDOFF = (-33.2, 34.9)
M2_2242_STANDOFF = (-10.4, -35.95)
# Pin header holes: 2x6 front panel header and 1x2 header on the left edge.
HEADER_PINS = [(x, 10.41 + 2 * k) for x in (-49.2, -47.2) for k in range(6)] + [
    (-49.2, 4.75), (-49.2, 6.75)
]

# Cooler
PLATE_TOP = 5.75
PIPE_TOP = 7.01  # the heat pipes, the fin base and the blower meet here
FIN_BASE = (-40.84, 41.06, 28.38, 36.38)  # x0, x1, y0, y1
FIN_Y1 = 51.38
FIN_TOP = 17.62
FAN_CENTER = (7.1, -14.0)
FAN_TOP = 17.12
FAN_WALL = 0.5
# Blower scroll outline: straight side walls, joined by a smooth curve around the front.
# The outlet (the straight back edge) faces the fins.
FAN_LEFT_WALL = [(-40.9, 28.2), (-42.3, 18.7), (-44.2, -20.7)]
FAN_SCROLL = [
    (-44.2, -20.7), (-39.5, -27.2), (-27.2, -39.4), (-17.8, -47.6),
    (-7.6, -50.2), (4.0, -50.2), (15.2, -50.0), (25.2, -44.9), (32.2, -38.9),
    (40.3, -27.8), (43.9, -18.0),
]
FAN_OUTLET_Y = 28.2
FAN_OUTLET_X = (-40.9, 41.2)
POWER_HEADER = (-43.2, -31.0, -41.98, -32.32, 0.21, 10.42)  # the blower has a notch for it


@dataclass
class NucParams:
    # Parts to include
    cooler: str = field(default="full", metadata={"choices": ("full", "heatsink", "none")})
    ram_sticks: int = 2  # DDR4 SO-DIMM modules: 0, 1 or 2
    ssd_2280: bool = True  # M.2 2280 NVMe SSD in the main slot
    wlan: bool = True  # M.2 2230 Wi-Fi card (above the SSD)
    ssd_2242: bool = False  # M.2 2242 SATA SSD in the second slot
    port_openings: bool = True  # cut the openings into the port shells
    # Fin stack
    fin_count: int = 78
    fin_pitch: float = 1.05
    fin_t: float = 0.2
    # Heat pipes (two flat pipes)
    pipe_w: float = 7.0
    pipe_t: float = 2.0
    # Blower
    impeller_blades: int = 31
    inlet_d: float = 40.0
    # Look
    pcb_color: str = field(default="green", metadata={"choices": tuple(PCB_COLORS)})

    def __post_init__(self):
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if f.type is float and value <= 0:
                raise ValueError(f"{f.name} must be positive, got {value}")
            choices = f.metadata.get("choices")
            if choices and value not in choices:
                raise ValueError(f"{f.name} must be one of {choices}, got {value!r}")
        if self.ram_sticks not in (0, 1, 2):
            raise ValueError("ram_sticks must be 0, 1 or 2")
        if self.fin_count < 2:
            raise ValueError("fin_count must be 2 or more")
        if self.fin_t >= self.fin_pitch:
            raise ValueError("fin_t must be smaller than fin_pitch")
        base_w = FIN_BASE[1] - FIN_BASE[0]
        if (self.fin_count - 1) * self.fin_pitch + self.fin_t > base_w:
            raise ValueError(f"the fins are wider than the fin base ({base_w:.2f} mm)")
        if not 3.0 <= self.pipe_w <= 8.0:
            raise ValueError("pipe_w must be between 3 and 8 mm")
        # The pipe must reach down into the copper plate, and stay above the dual USB jack.
        if not PIPE_TOP - PLATE_TOP < self.pipe_t <= 2.0:
            raise ValueError(f"pipe_t must be between {PIPE_TOP - PLATE_TOP:.2f} and 2.0 mm")
        if self.impeller_blades < 3:
            raise ValueError("impeller_blades must be 3 or more")
        if not 10.0 <= self.inlet_d <= 54.0:
            raise ValueError("inlet_d must be between 10 and 54 mm")


# ---------------------------------------------------------------- helpers


def box(x0, x1, y0, y1, z0, z1) -> cq.Solid:
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))


def cyl(x, y, z0, z1, d) -> cq.Solid:
    return cq.Solid.makeCylinder(d / 2, z1 - z0, cq.Vector(x, y, z0))


def y_prism(sketch, cx, cz, y0, y1) -> cq.Solid:
    """Extrude a 2D shape drawn in the XZ plane (centered on cx, cz) from y0 to y1."""
    wp = cq.Workplane("XZ", origin=(cx, y1, cz))  # the XZ normal is -Y
    return sketch(wp).extrude(y1 - y0).val()


def fuse(solids) -> cq.Shape:
    """Fuse all shapes in one step. Fusing one by one can leave pieces unjoined."""
    first, *rest = solids
    return first.fuse(*rest).clean() if rest else first


def cut(shape, tools) -> cq.Shape:
    return shape.cut(*tools).clean() if tools else shape


def rounded_polygon(corners, z: float = 0.0) -> cq.Wire:
    """Closed wire through (x, y, r) corners. r > 0 rounds that corner."""
    n = len(corners)
    corner_pts = []  # (arc start, arc middle or None, arc end)
    for i, (x, y, r) in enumerate(corners):
        p = cq.Vector(x, y, z)
        if r == 0:
            corner_pts.append((p, None, p))
            continue
        u1 = (cq.Vector(*corners[i - 1][:2], z) - p).normalized()
        u2 = (cq.Vector(*corners[(i + 1) % n][:2], z) - p).normalized()
        half = math.acos(max(-1.0, min(1.0, u1.dot(u2)))) / 2
        bisector = (u1 + u2).normalized()
        center = p + bisector * (r / math.sin(half))
        d = r / math.tan(half)
        corner_pts.append((p + u1 * d, center - bisector * r, p + u2 * d))
    edges = []
    for i, (start, mid, end) in enumerate(corner_pts):
        if mid is not None:
            edges.append(cq.Edge.makeThreePointArc(start, mid, end))
        nxt = corner_pts[(i + 1) % n][0]
        if (nxt - end).Length > 1e-6:
            edges.append(cq.Edge.makeLine(end, nxt))
    return cq.Wire.assembleEdges(edges)


class Opening:
    """Port openings cut into one face of a port shell. Inserts (tongues, pins) sit inside."""

    def __init__(self, p: NucParams, y_face: float, depth: float):
        self.p = p
        self.y_face = y_face
        self.inward = -1 if y_face > 0 else 1  # back ports face +Y, front ports face -Y
        self.depth = depth
        self.cuts: list[cq.Solid] = []
        self.inserts: list[cq.Solid] = []

    def span(self, front_gap: float = 0.0) -> tuple[float, float]:
        """Y range from the opening's back wall to front_gap inside the face."""
        a = self.y_face + self.inward * front_gap
        b = self.y_face + self.inward * self.depth
        return min(a, b), max(a, b)

    def hole(self, sketch, cx, cz):
        y0, y1 = self.span()
        self.cuts.append(y_prism(sketch, cx, cz, y0 - 0.1, y1 + 0.1))

    def insert(self, solid_fn):
        """solid_fn(y0, y1) builds an insert from the back wall to 1 mm inside the face."""
        self.inserts.append(solid_fn(*self.span(front_gap=1.0)))

    def apply(self, shell: cq.Shape) -> tuple[cq.Shape, list[cq.Solid]]:
        if not self.p.port_openings:
            return shell, []
        return cut(shell, self.cuts), self.inserts


# ---------------------------------------------------------------- board


def make_pcb(p: NucParams) -> cq.Shape:
    pcb = cq.Solid.extrudeLinear(cq.Face.makeFromWires(rounded_polygon(PCB_OUTLINE)),
                                 cq.Vector(0, 0, PCB_T))
    z0, z1 = -1.0, PCB_T + 1.0
    tools = []
    for x in (48.8, -48.8):  # half-round notches on both side edges
        tools.append(box(min(x, 1.5 * x), max(x, 1.5 * x), SIDE_NOTCH_Y - 2, SIDE_NOTCH_Y + 2, z0, z1))
        tools.append(cyl(x, SIDE_NOTCH_Y, z0, z1, 4.0))
    # Mount holes and slots (x, y, length, width, angle)
    for x, y, length, width, angle in [
        (-47.5, -45.75, 4.4, 3.9, 0), (-47.5, 46.125, 6.95, 4.3, 90), (47.5, 46.125, 6.45, 3.9, 90),
    ]:
        tools.append(cq.Workplane("XY", origin=(0, 0, z0)).center(x, y)
                     .slot2D(length, width, angle).extrude(z1 - z0).val())
    tools.append(cyl(47.5, -45.75, z0, z1, 3.9))
    tools += [cyl(x, y, z0, z1, 3.8) for x, y in HEATSINK_HOLES]
    tools.append(cyl(*M2_2280_STANDOFF, z0, z1, 4.3))
    tools.append(cyl(*M2_2242_STANDOFF, z0, z1, 3.4))
    tools += [cyl(x, y, z0, z1, 0.96) for x, y in HEADER_PINS]
    return cut(pcb, tools)


def make_ports(p: NucParams, pcb: cq.Shape) -> dict[str, list]:
    """Back and front panel connectors. Returns shells and inserts by color group."""
    shells, black, blue = [], [], []

    def add(shell, opening: Opening, insert_group=None):
        shell, inserts = opening.apply(shell)
        shells.append(shell)
        (insert_group if insert_group is not None else black).extend(inserts)

    # HDMI 2.0b x2 (bottom face, back)
    for x0 in (-43.31, 13.99):
        x1, cx, cz = x0 + 16.2, x0 + 8.1, -6.19
        shell = fuse([box(x0, x1, 43.34, 53.65, -9.44, -2.94),
                      box(x0 + 1, x1 - 1, 43.34, 49.0, -2.94, 0.0)])
        o = Opening(p, 53.65, 9.0)
        o.hole(lambda w: w.polyline([(-7, 2.25), (7, 2.25), (7, -0.95), (5.7, -2.25),
                                     (-5.7, -2.25), (-7, -0.95)]).close(), cx, cz)
        o.insert(lambda y0, y1: box(cx - 5.5, cx + 5.5, y0, y1, cz - 0.2, cz + 1.3))
        add(shell, o)

    # Dual USB-A 3.2 (through the board notch, back)
    x0, x1, cx = -22.33, -6.47, -14.4
    shell = fuse([box(x0, x1, 37.2, 53.3, -10.65, 4.95), box(-21.8, -7.0, 29.82, 37.2, -4.0, 0.0)])
    shell = shell.cut(pcb).clean()  # side slots where the board edges pass
    o = Opening(p, 53.3, 12.0)
    for cz in (-6.75, 1.05):
        o.hole(lambda w: w.rect(12.5, 5.12), cx, cz)
        o.insert(lambda y0, y1, cz=cz: box(cx - 5.6, cx + 5.6, y0, y1, cz + 0.2, cz + 2.04))
    add(shell, o, blue)

    # RJ45 2.5 GbE (through the board notch, back)
    cx, cz = 3.81, -2.66
    shell = fuse([box(-5.72, 13.34, 40.95, 53.08, -8.08, 3.97),
                  box(-4.3, 11.9, 38.4, 40.95, -8.08, 0.0)])
    shell = shell.cut(pcb).clean()
    o = Opening(p, 53.08, 14.0)
    o.hole(lambda w: w.rect(11.7, 7.0), cx, cz)
    o.hole(lambda w: w.rect(6.0, 1.7), cx, cz + 4.3)
    add(shell, o)

    # DC input jack (bottom face, back)
    cx, cz = 38.9, -5.0
    o = Opening(p, 53.3, 10.0)
    o.hole(lambda w: w.circle(3.2).circle(1.0), cx, cz)
    add(box(33.9, 43.9, 40.14, 53.3, -10.05, 0.0), o)

    # Thunderbolt 4 USB-C x2 (top face, back)
    for x0 in (-40.05, 17.25):
        cx, cz = x0 + 4.84, (PCB_T + 5.11) / 2
        shell = y_prism(lambda w: w.slot2D(9.68, 5.11 - PCB_T), cx, cz, 44.53, 53.8)
        o = Opening(p, 53.8, 7.0)
        o.hole(lambda w: w.slot2D(8.34, 2.56), cx, cz)
        o.insert(lambda y0, y1: box(cx - 3.3, cx + 3.3, y0, y1, cz - 0.35, cz + 0.35))
        add(shell, o)

    # Front USB-A 3.2 x2 (bottom face, front)
    for x0 in (-21.84, -3.84):
        cx, cz = x0 + 6.9, -3.23
        o = Opening(p, -53.68, 12.0)
        o.hole(lambda w: w.rect(12.5, 5.12), cx, cz)
        o.insert(lambda y0, y1: box(cx - 5.6, cx + 5.6, y0, y1, cz + 0.2, cz + 2.04))
        add(box(x0, x0 + 13.8, -53.68, -39.78, -6.46, 0.0), o, blue)

    # 3.5 mm audio jack (bottom face, front)
    cx, cz = 19.05, -2.94
    shell = fuse([box(14.3, 23.8, -52.05, -41.15, -5.87, 0.0),
                  y_prism(lambda w: w.circle(2.5), cx, cz, -54.55, -52.05 + 0.5)])
    o = Opening(p, -54.55, 12.0)
    o.hole(lambda w: w.circle(1.8), cx, cz)
    black.append(shell if not p.port_openings else cut(shell, o.cuts))

    # Power button and status LEDs (bottom face, front)
    black.append(fuse([box(30.79, 38.65, -51.45, -47.95, -3.6, 0.0),
                       box(33.2, 36.3, -52.3, -51.45 + 0.2, -2.9, -0.7)]))
    leds = [box(26.82, 30.0, -50.07, -48.01, -1.2, 0.0), box(39.6, 42.78, -50.1, -48.05, -1.2, 0.0)]
    return {"port_shells": shells, "port_inserts": black, "usb3_tongues": blue, "leds": leds}


def make_headers(p: NucParams) -> dict[str, list]:
    """Internal connectors and headers."""
    white = [
        box(-49.77, -45.48, -41.46, -31.30, -4.87, 0.0),  # 4-pin wire-to-board
        box(-49.77, -45.48, -30.21, -20.05, -4.87, 0.0),  # 4-pin wire-to-board
        box(-49.90, -45.69, -18.75, -11.49, -4.90, 0.0),  # 2-pin right angle
        box(-49.77, -45.52, -9.98, 1.02, -4.90, 0.0),  # 5-pin wire-to-board
        box(38.80, 48.96, -36.90, -32.61, PCB_T, 6.43),  # CPU fan header (top face)
    ]
    black = [
        box(-21.60, -7.10, 11.03, 14.40, -4.65, 0.0),  # SATA FPC connector
        box(-4.74, 12.06, 25.24, 35.14, PCB_T, 4.16),  # LAN magnetics (top face)
    ]
    # 2x2 internal power header (top face) with 4 pin sockets
    x0, x1, y0, y1, _, z1 = POWER_HEADER
    power = box(x0, x1, y0, y1, PCB_T, z1)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    black.append(cut(power, [box(cx + dx - 1.8, cx + dx + 1.8, cy + dy - 1.8, cy + dy + 1.8, z1 - 7, z1 + 1)
                             for dx in (-2.1, 2.1) for dy in (-2.1, 2.1)]))
    # Pin headers: plastic base on the bottom face, square pins through the board
    pins = []
    for (xs, ys) in ([(-49.2, -47.2), [10.41 + 2 * k for k in range(6)]], [(-49.2,), (4.75, 6.75)]):
        black.append(box(min(xs) - 1.0, max(xs) + 1.0, min(ys) - 1.0, max(ys) + 1.0, -2.5, 0.0))
        pins += [box(x - 0.32, x + 0.32, y - 0.32, y + 0.32, -6.1, 2.4) for x in xs for y in ys]
    black.append(box(-16.67, -10.92, 24.54, 28.74, -2.5, 0.0))  # 1x3 header
    pins += [box(x - 0.32, x + 0.32, 26.32, 26.96, -6.84, -2.5) for x in (-15.8, -13.8, -11.8)]
    black = [b.cut(*pins).clean() for b in black]  # holes for the pins in the plastic bases
    return {"headers_white": white, "headers_black": black, "pins": pins}


def make_soc() -> dict[str, list]:
    """Intel Core (Raptor Lake-P) BGA package: substrate and die."""
    substrate = box(-13.57, 11.43, -30.57, 19.43, PCB_T, 2.2)
    die = box(-6.77, 4.63, -19.45, 8.31, 2.2, 2.75)
    return {"soc_substrate": [substrate], "soc_die": [die]}


# ---------------------------------------------------------------- memory and storage


def make_sodimm(x0: float, z_mid: float) -> tuple[cq.Solid, list[cq.Solid]]:
    """DDR4 SO-DIMM (30 x 69.6 mm). The gold fingers are at the +X edge."""
    y0, y1 = -38.25, 31.35
    board = box(x0, x0 + 30.0, y0, y1, z_mid - 0.6, z_mid + 0.6)
    chips = [box(x0 + 7, x0 + 17, y, y + 11, z, z + 1.2)
             for y in (y0 + 4, y0 + 20, y0 + 36, y0 + 52)
             for z in (z_mid - 1.8, z_mid + 0.6)]
    return board, chips


def make_memory(p: NucParams) -> dict[str, list]:
    # Two stacked slots. The near slot sits against the board. The far slot is below it.
    slots = [
        # (stick x0, stick z mid, socket body x range, socket z0, latch x range, latch y)
        (1.84, -3.48, (29.5, 36.2), -5.35, (10.41, 16.41), ((-40.11, -38.25), (31.35, 33.21))),
        (14.29, -7.48, (42.0, 48.66), -9.52, (24.3, 29.0), ((-41.67, -38.25), (31.35, 34.77))),
    ]
    sockets, boards, chips = [], [], []
    for i, (sx0, z_mid, (bx0, bx1), sz0, (lx0, lx1), latch_y) in enumerate(slots):
        ly0, ly1 = latch_y[0][0], latch_y[1][1]
        body = box(bx0, bx1, ly0, ly1, sz0, 0.0)
        latches = [box(lx0, lx1, a, b, sz0, 0.0) for a, b in latch_y]
        if i < p.ram_sticks:
            board, stick_chips = make_sodimm(sx0, z_mid)
            body = body.cut(board).clean()
            boards.append(board)
            chips += stick_chips
        sockets += [body, *latches]
    return {"dimm_sockets": sockets, "dimm_boards": boards, "chips": chips}


def make_m2_card(x0, y0, y1, pcb_z0, chip_z0, chip_z1, notch_y, chip_ys) -> tuple:
    """M.2 card, 22 mm wide, with a half-round notch for the screw at notch_y."""
    board = box(x0, x0 + 22.0, y0, y1, pcb_z0, pcb_z0 + 0.8)
    board = board.cut(cyl(x0 + 11.0, notch_y, pcb_z0 - 1, pcb_z0 + 2, 3.5)).clean()
    chips = [box(x0 + 2.5, x0 + 19.5, a, b, chip_z0, chip_z1) for a, b in chip_ys]
    return board, chips


def make_storage(p: NucParams) -> dict[str, list]:
    connectors, standoffs, boards, chips = [], [], [], []
    sx, sy = M2_2280_STANDOFF
    # M.2 2280 key M slot and card. The Wi-Fi card is stacked above the SSD.
    ssd_conn = box(-44.13, -22.27, -47.95, -40.85, -8.62, 0.0)
    wlan_conn = box(-44.13, -22.27, 0.75, 9.2, -4.13, 0.0)
    standoffs.append(fuse([cyl(sx, sy, -2.57, 0.0, 5.5), cyl(sx, sy, 0.0, 1.36, 4.2)]))
    standoffs.append(fuse([cyl(sx, sy, -6.68, -3.37, 5.0), cyl(sx, sy, -7.48, -3.37, 2.5)]))
    if p.ssd_2280:
        board, c = make_m2_card(-44.2, -45.1, 34.9, -7.48, -9.0, -7.48, 34.9,
                                [(-38.5, -26.5), (-22.0, -4.0), (1.0, 19.0)])
        ssd_conn = ssd_conn.cut(board).clean()
        boards.append(board)
        chips += c
        standoffs.append(cyl(sx, sy, -8.3, -7.48, 4.0))  # M2 screw head
    if p.wlan:
        board, c = make_m2_card(-44.2, 4.9, 34.9, -3.37, -4.87, -3.37, 34.9, [(12.0, 30.0)])
        wlan_conn = wlan_conn.cut(board).clean()
        boards.append(board)
        chips += c
    connectors += [ssd_conn, wlan_conn]

    # M.2 2242 key B slot (SATA). The standoff nut is always on the board.
    tx, ty = M2_2242_STANDOFF
    conn_2242 = box(-21.35, 0.55, 1.8, 8.85, -8.6, 0.0)
    standoffs.append(fuse([cyl(tx, ty, -7.46, 0.0, 5.5), cyl(tx, ty, 0.0, 1.36, 3.3)]))
    if p.ssd_2242:
        board, c = make_m2_card(-21.62, -35.95, 6.05, -8.26, -7.46, -5.3, -35.95,
                                [(-30.0, -12.0), (-9.0, 0.5)])
        conn_2242 = conn_2242.cut(board).clean()
        boards.append(board)
        chips += c
        standoffs.append(cyl(tx, ty, -9.06, -8.26, 4.0))
    connectors.append(conn_2242)
    return {"m2_connectors": connectors, "m2_standoffs": standoffs, "m2_boards": boards,
            "chips": chips}


# ---------------------------------------------------------------- cooler


def make_pipe(p: NucParams, side: int) -> cq.Solid:
    """Flat heat pipe: from the front of the copper plate to the back, then out along the fins."""
    x = side * (p.pipe_w / 2 + 0.1)
    y_turn = (FIN_BASE[2] + FIN_BASE[3]) / 2
    x_end = FIN_BASE[1] if side > 0 else FIN_BASE[0]
    r = 8.0
    zc = PIPE_TOP - p.pipe_t / 2
    c = cq.Vector(x + side * r, y_turn - r, zc)  # bend center
    path = cq.Wire.assembleEdges([
        cq.Edge.makeLine(cq.Vector(x, -26.5, zc), cq.Vector(x, y_turn - r, zc)),
        cq.Edge.makeThreePointArc(cq.Vector(x, y_turn - r, zc),
                                  c + cq.Vector(-side * r * math.sqrt(0.5), r * math.sqrt(0.5), 0),
                                  cq.Vector(x + side * r, y_turn, zc)),
        cq.Edge.makeLine(cq.Vector(x + side * r, y_turn, zc), cq.Vector(x_end, y_turn, zc)),
    ])
    profile = cq.Workplane("XZ", origin=(x, -26.5, zc)).slot2D(p.pipe_w, p.pipe_t)
    return profile.sweep(cq.Workplane().add(path), transition="round").val()


def clip_parts(side: int) -> list[cq.Solid]:
    """Footprint of one spring clip: a bar through two screw holes and an arm to the plate center."""
    hx = 18.95 if side > 0 else -21.05
    z0, z1 = 5.24, 5.65
    bar = box(hx - 2.25, hx + 2.25, -30.37, 24.13, z0, z1)
    arm = (box(9.75, hx - 2.25, -8.5, -2.5, z0, z1) if side > 0
           else box(hx + 2.25, -11.85, -8.5, -2.5, z0, z1))
    return [bar, arm]


def make_heatsink(p: NucParams) -> dict[str, list]:
    x0, x1, y0, y1 = FIN_BASE
    # Copper plate with grooves for the clips.
    plate = box(-23.2, 21.1, -21.62, 15.38, 2.75, PLATE_TOP)
    grooves = [box(*_bounds(s)[:4], 5.24, PLATE_TOP + 0.1) for side in (1, -1) for s in clip_parts(side)]
    plate = plate.cut(*grooves).clean()
    fin_base = box(x0, x1, y0, y1, PIPE_TOP, PIPE_TOP + 0.31)
    width = (p.fin_count - 1) * p.fin_pitch + p.fin_t
    fx0 = (x0 + x1) / 2 - width / 2
    fins = [box(fx0 + i * p.fin_pitch, fx0 + i * p.fin_pitch + p.fin_t, y0, FIN_Y1,
                PIPE_TOP + 0.31, FIN_TOP) for i in range(p.fin_count)]
    heatsink = fuse([plate, fin_base, make_pipe(p, 1), make_pipe(p, -1), *fins])

    clips, screws, springs, nuts = [], [], [], []
    for side in (1, -1):
        hx = 18.95 if side > 0 else -21.05
        clip = fuse(clip_parts(side))
        clips.append(clip.cut(*[cyl(hx, hy, 5, 6, 2.4) for hy in (-28.12, 21.88)]).clean())
    for x, y in HEATSINK_HOLES:
        screws.append(fuse([cyl(x, y, 4.09, 5.65, 2.0), cyl(x, y, 5.65, 6.45, 3.8)]))
        springs.append(cyl(x, y, 4.09, 5.24, 3.4).cut(cyl(x, y, 4.0, 5.3, 2.2)).clean())
        nuts.append(fuse([cyl(x, y, PCB_T, 4.09, 6.0), cyl(x, y, -1.1, PCB_T, 3.7)]))
    return {"heatsink": [heatsink], "clips": clips, "screws": screws + springs,
            "cooler_standoffs": nuts}


def _bounds(s: cq.Shape) -> tuple:
    b = s.BoundingBox()
    return b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax


def scroll_solid(z0: float, z1: float, inset: float) -> cq.Solid:
    """Blower outline between z0 and z1, moved inward by inset."""
    pts = [cq.Vector(x, y, z0) for x, y in FAN_SCROLL]
    left = [cq.Vector(x, y, z0) for x, y in FAN_LEFT_WALL]
    right = cq.Vector(FAN_OUTLET_X[1], FAN_OUTLET_Y, z0)
    # The spline ends follow the straight walls, so the curve does not bulge out.
    spline = cq.Edge.makeSpline(pts, tangents=[left[2] - left[1], right - pts[-1]])
    wire = cq.Wire.assembleEdges([
        spline, cq.Edge.makeLine(pts[-1], right), cq.Edge.makeLine(right, left[0]),
        cq.Edge.makeLine(left[0], left[1]), cq.Edge.makeLine(left[1], left[2]),
    ])
    if inset:
        wire = wire.offset2D(-inset, "arc")[0]
    return cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), cq.Vector(0, 0, z1 - z0))


def make_blower(p: NucParams) -> dict[str, list]:
    z0, z1, w = PIPE_TOP, FAN_TOP, FAN_WALL
    x0, x1, y0, y1, _, _ = POWER_HEADER
    outer = scroll_solid(z0, z1, 0).cut(box(x0 - 0.5, x1 + 0.5, y0 - 0.5, y1 + 0.5, z0 - 1, z1 + 1))
    inner = scroll_solid(z0 + w, z1 - w, 1.0).cut(
        box(x0 - 1.5, x1 + 1.5, y0 - 1.5, y1 + 1.5, z0 - 1, z1 + 1))
    fx, fy = FAN_CENTER
    housing = outer.cut(
        inner,
        box(FAN_OUTLET_X[0] + 1.0, FAN_OUTLET_X[1] - 1.0, 20.0, FAN_OUTLET_Y + 1, z0 + w, z1 - w),
        cyl(fx, fy, z1 - w - 0.1, z1 + 0.1, p.inlet_d),
    )  # clean() here makes the solid invalid

    # Impeller: a hub cup on a disk, with backward-leaning straight blades.
    hub = [cyl(fx, fy, 7.9, 8.3, 36.0), cyl(fx, fy, 8.3, 15.4, 24.0)]
    blades = []
    for k in range(p.impeller_blades):
        blade = box(-6.5, 6.5, -0.25, 0.25, 8.0, 16.1)
        blade = blade.rotate(cq.Vector(), cq.Vector(0, 0, 1), -25).translate((22.0, 0, 0))
        blade = blade.rotate(cq.Vector(), cq.Vector(0, 0, 1), 360.0 * k / p.impeller_blades)
        blades.append(blade.translate((fx, fy, 0)))
    impeller = fuse(hub + blades)
    return {"blower_housing": [housing], "impeller": [impeller]}


# ---------------------------------------------------------------- assembly

GROUP_COLORS = {
    "port_shells": METAL, "port_inserts": BLACK, "usb3_tongues": USB3_BLUE, "leds": LED_WHITE,
    "headers_white": IVORY, "headers_black": BLACK, "pins": GOLD,
    "soc_substrate": DIMM_GREEN, "soc_die": DIE_GRAY,
    "dimm_sockets": BLACK, "dimm_boards": DIMM_GREEN, "chips": BLACK,
    "m2_connectors": BLACK, "m2_standoffs": METAL, "m2_boards": (0.12, 0.12, 0.14),
    "heatsink": COPPER, "clips": METAL, "screws": DARK, "cooler_standoffs": METAL,
    "blower_housing": DARK, "impeller": BLACK,
}


def build_nuc(p: NucParams) -> cq.Assembly:
    pcb = make_pcb(p)
    groups: dict[str, list] = {}
    parts = [make_ports(p, pcb), make_headers(p), make_soc(), make_memory(p), make_storage(p)]
    if p.cooler != "none":
        parts.append(make_heatsink(p))
    if p.cooler == "full":
        parts.append(make_blower(p))
    for part in parts:
        for name, solids in part.items():
            groups.setdefault(name, []).extend(solids)

    assy = cq.Assembly(name="nuc13pro")
    assy.add(pcb, name="pcb", color=cq.Color(*PCB_COLORS[p.pcb_color]))
    for name, solids in groups.items():
        if solids:
            shape = solids[0] if len(solids) == 1 else cq.Compound.makeCompound(solids)
            assy.add(shape, name=name, color=cq.Color(*GROUP_COLORS[name]))
    return assy


def add_param_args(parser: argparse.ArgumentParser, cls) -> None:
    """One flag per dataclass field. Default None marks flags the user did not give."""
    for f in dataclasses.fields(cls):
        flag = "--" + f.name.replace("_", "-")
        if f.type is bool:
            parser.add_argument(flag, dest=f.name, action=argparse.BooleanOptionalAction,
                                default=None, help=f"default: {f.default}")
        else:
            default = f"{f.default:.4g}" if f.type is float else f.default
            parser.add_argument(flag, dest=f.name, type=f.type, default=None,
                                choices=f.metadata.get("choices"), help=f"default: {default}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Build an Intel NUC 13 Pro board (no case) STEP model.")
    add_param_args(parser, NucParams)
    parser.add_argument("-o", "--output", default="nuc13pro.step", help="STEP output path")
    args = vars(parser.parse_args())
    output = args.pop("output")
    try:
        p = NucParams(**{k: v for k, v in args.items() if v is not None})
    except ValueError as e:
        parser.error(str(e))

    assy = build_nuc(p)
    assy.export(output)
    bb = assy.toCompound().BoundingBox()
    print(f"Wrote {output}")
    print(f"Parts: {', '.join(child.name for child in assy.children)}")
    print(f"Size: X {bb.xlen:.2f}  Y {bb.ylen:.2f}  Z {bb.zlen:.2f} mm "
          f"(Z {bb.zmin:.2f} to {bb.zmax:.2f})")


if __name__ == "__main__":
    main()
