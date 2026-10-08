"""Parametric drone propeller.

Each blade is a smooth loft through NACA 4-digit airfoil sections. The blade
angle at radius r follows the pitch: angle = atan(pitch / (2 * pi * r)), capped
at max_twist near the hub. The chord grows from the root to a maximum, then
shrinks toward the tip.

The Z axis is the rotation axis. Z = 0 is the hub bottom face (the motor side).
Thrust points to +Z. "ccw" means the prop turns counter-clockwise when viewed
from +Z (from above). All units are mm.

Usage:
    uv run python prop.py                              # 5.1 x 4.3 inch, 3 blades
    uv run python prop.py --spec 5x4.3x3 --direction cw -o prop_cw.step
    uv run python prop.py --spec 3x3x2 --hub-d 9 --max-chord 9 -o prop_3in.step
"""

import argparse
import dataclasses
import math
from dataclasses import dataclass, field

import cadquery as cq

INCH = 25.4
Z_AXIS = cq.Vector(0, 0, 1)
ORIGIN = cq.Vector(0, 0, 0)


@dataclass
class PropParams:
    # Size
    diameter: float = 5.1 * INCH  # tip-to-tip diameter
    pitch: float = 4.3 * INCH  # geometric pitch (distance moved per turn)
    blade_count: int = 3
    direction: str = field(default="ccw", metadata={"choices": ("ccw", "cw")})
    # Blade plan form
    root_chord: float = 7.0  # chord at the hub edge
    max_chord: float = 13.0  # largest chord
    max_chord_pos: float = 0.45  # radius of largest chord, as a fraction of the tip radius
    tip_chord: float = 5.0  # chord at the tip
    sweep: float = 4.0  # tip moves back by this distance (0 = straight blade)
    max_twist: float = 50.0  # largest blade angle near the hub, in degrees
    # Airfoil (NACA 4-digit)
    camber: float = 0.04  # max camber, as a fraction of the chord
    camber_pos: float = 0.4  # position of max camber, as a fraction of the chord
    thickness_root: float = 0.14  # thickness / chord at the root
    thickness_tip: float = 0.08  # thickness / chord at the tip
    te_thickness: float = 0.3  # trailing edge thickness
    pivot: float = 0.35  # chord position placed on the blade axis (fraction of chord)
    # Hub
    hub_d: float = 12.0
    hub_h: float = 6.5
    bore_d: float = 5.0  # motor shaft hole (0 = none)
    hub_chamfer: float = 0.5
    # Model quality
    sections: int = 14  # airfoil sections per blade
    airfoil_points: int = 25  # points per airfoil surface

    @property
    def tip_r(self) -> float:
        return self.diameter / 2

    @property
    def hub_r(self) -> float:
        return self.hub_d / 2

    @property
    def root_r(self) -> float:
        """Radius of the first section. It is inside the hub, so the blade joins the hub."""
        return (max(self.bore_d, 0.0) / 2 + self.hub_r) / 2

    def blade_angle(self, r: float) -> float:
        """Blade angle in radians at radius r."""
        return min(math.atan2(self.pitch, 2 * math.pi * r), math.radians(self.max_twist))

    def chord(self, r: float) -> float:
        t = r / self.tip_r
        t_hub = self.hub_r / self.tip_r
        if t <= t_hub:
            return self.root_chord
        if t <= self.max_chord_pos:
            s = (t - t_hub) / (self.max_chord_pos - t_hub)
            return self.root_chord + (self.max_chord - self.root_chord) * math.sin(math.pi / 2 * s)
        s = (t - self.max_chord_pos) / (1 - self.max_chord_pos)
        return self.tip_chord + (self.max_chord - self.tip_chord) * math.cos(math.pi / 2 * s)

    def thickness(self, r: float) -> float:
        s = (r - self.root_r) / (self.tip_r - self.root_r)
        return self.thickness_root + (self.thickness_tip - self.thickness_root) * s

    def __post_init__(self):
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if f.type is float and value < 0:
                raise ValueError(f"{f.name} must not be negative, got {value}")
        for name in ("diameter", "pitch", "root_chord", "max_chord", "tip_chord", "hub_d", "hub_h"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.direction not in ("ccw", "cw"):
            raise ValueError("direction must be 'ccw' or 'cw'")
        if self.blade_count < 1:
            raise ValueError("blade_count must be 1 or more")
        if self.sections < 3 or self.airfoil_points < 8:
            raise ValueError("sections must be >= 3 and airfoil_points >= 8")
        if not 0 < self.max_twist < 90:
            raise ValueError("max_twist must be between 0 and 90 degrees")
        if not 0 < self.pivot < 1:
            raise ValueError("pivot must be between 0 and 1")
        if not 0 < self.camber_pos < 1 or self.camber >= 0.1:
            raise ValueError("camber_pos must be between 0 and 1, and camber below 0.1")
        if not (0 < self.thickness_tip < 0.3 and 0 < self.thickness_root < 0.3):
            raise ValueError("airfoil thickness must be between 0 and 0.3")
        if self.hub_r >= self.tip_r * 0.5:
            raise ValueError("hub_d is too large for the diameter")
        if self.bore_d >= self.hub_d - 2.0:
            raise ValueError("bore_d leaves less than 1 mm hub wall")
        if not self.hub_r / self.tip_r < self.max_chord_pos < 1:
            raise ValueError("max_chord_pos must be between the hub edge and the tip")
        if self.max_chord < max(self.root_chord, self.tip_chord):
            raise ValueError("max_chord must be at least root_chord and tip_chord")
        if self.hub_chamfer * 2 >= self.hub_h:
            raise ValueError("hub_chamfer is too large for hub_h")
        # The blade root must stay inside the hub height.
        a = self.blade_angle(self.hub_r)
        c = self.root_chord
        root_h = c * math.sin(a) + self.thickness_root * c * math.cos(a) + self.te_thickness
        if root_h > self.hub_h:
            raise ValueError(
                f"blade root is {root_h:.1f} mm tall and sticks out of the hub "
                f"({self.hub_h} mm): reduce root_chord or max_twist, or increase hub_h"
            )


def naca4(p: PropParams, chord: float, thickness: float) -> list[tuple[float, float]]:
    """Airfoil points (u, v) in mm. u runs along the chord from the leading edge,
    v is up. Order: upper trailing edge -> leading edge -> lower trailing edge."""
    m, mp = p.camber, p.camber_pos
    n = p.airfoil_points
    upper, lower = [], []
    for i in range(n):
        x = (1 - math.cos(math.pi * i / (n - 1))) / 2  # cosine spacing, dense at both ends
        # Closed trailing edge form (-0.1036), then add a fixed trailing edge thickness.
        yt = 5 * thickness * (
            0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4
        )
        yt = yt * chord + p.te_thickness / 2 * x
        if x < mp:
            yc = m / mp**2 * (2 * mp * x - x**2)
            dyc = 2 * m / mp**2 * (mp - x)
        else:
            yc = m / (1 - mp) ** 2 * ((1 - 2 * mp) + 2 * mp * x - x**2)
            dyc = 2 * m / (1 - mp) ** 2 * (mp - x)
        th = math.atan(dyc)
        xc, yc = x * chord, yc * chord
        upper.append((xc - yt * math.sin(th), yc + yt * math.cos(th)))
        lower.append((xc + yt * math.sin(th), yc - yt * math.cos(th)))
    return upper[::-1] + lower[1:]


def section_wire(p: PropParams, r: float) -> cq.Wire:
    """Airfoil section at radius r for a ccw blade on the +X axis.

    The blade moves toward +Y. The leading edge is at +Y and is higher than the
    trailing edge by the blade angle, so the blade pushes air toward -Z.
    """
    chord = p.chord(r)
    a = p.blade_angle(r)
    s = (r - p.root_r) / (p.tip_r - p.root_r)
    y_sweep = -p.sweep * s**2
    z_mid = p.hub_h / 2
    # Along the rotation direction the pivot point is on the blade axis.
    # In height the mid chord is at the hub middle, so the blade stays centered on the hub.
    u0 = p.pivot * chord
    pts = []
    for u, v in naca4(p, chord, p.thickness(r)):
        y = y_sweep - (u - u0) * math.cos(a) - v * math.sin(a)
        z = z_mid - (u - chord / 2) * math.sin(a) + v * math.cos(a)
        pts.append(cq.Vector(r, y, z))
    surface = cq.Edge.makeSpline(pts)
    trailing_edge = cq.Edge.makeLine(pts[-1], pts[0])
    return cq.Wire.assembleEdges([surface, trailing_edge])


def make_blade(p: PropParams) -> cq.Solid:
    n = p.sections
    radii = [p.root_r + (p.tip_r - p.root_r) * i / (n - 1) for i in range(n)]
    return cq.Solid.makeLoft([section_wire(p, r) for r in radii], ruled=False)


def make_hub(p: PropParams) -> cq.Workplane:
    hub = cq.Workplane("XY").circle(p.hub_r).extrude(p.hub_h)
    if p.hub_chamfer > 0:
        hub = hub.edges().chamfer(p.hub_chamfer)
    return hub


def build_prop(p: PropParams) -> cq.Workplane:
    blade = make_blade(p)
    blades = [blade.rotate(ORIGIN, Z_AXIS, 360.0 * k / p.blade_count) for k in range(p.blade_count)]
    # Fuse all blades in one step. Fusing them one by one can leave a blade unjoined.
    prop = cq.Workplane("XY").add(make_hub(p).val().fuse(*blades).clean())
    if p.bore_d > 0:
        bore = cq.Workplane("XY").workplane(offset=-1).circle(p.bore_d / 2).extrude(p.hub_h + 2)
        prop = prop.cut(bore)
    if p.direction == "cw":
        prop = prop.mirror("XZ")
    return prop


def parse_spec(spec: str) -> dict:
    """'5.1x4.3x3' (inches, inches, blades) -> diameter, pitch and blade_count."""
    parts = spec.lower().split("x")
    if len(parts) not in (2, 3):
        raise ValueError(f"bad --spec '{spec}', expected e.g. 5.1x4.3x3")
    out = {"diameter": float(parts[0]) * INCH, "pitch": float(parts[1]) * INCH}
    if len(parts) == 3:
        out["blade_count"] = int(parts[2])
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Build a drone propeller STEP model.")
    parser.add_argument(
        "--spec",
        help="size in inches as DIAMETERxPITCH[xBLADES], e.g. 5.1x4.3x3. Other flags override it.",
    )
    for f in dataclasses.fields(PropParams):
        flag = "--" + f.name.replace("_", "-")
        # Default None marks flags the user did not give.
        parser.add_argument(
            flag, type=f.type, default=None, choices=f.metadata.get("choices"),
            help=f"default: {f.default:.4g}" if f.type is float else f"default: {f.default}",
        )
    parser.add_argument("-o", "--output", default="prop.step", help="output path (.step or .stl)")
    args = vars(parser.parse_args())
    output = args.pop("output")
    spec = args.pop("spec")

    try:
        values = parse_spec(spec) if spec else {}
        values.update({k: v for k, v in args.items() if v is not None})
        params = PropParams(**values)
    except ValueError as e:
        parser.error(str(e))

    prop = build_prop(params)
    # A fine mesh for STL, so thin blade edges stay smooth.
    cq.exporters.export(prop, output, tolerance=0.02, angularTolerance=0.1)
    bb = prop.val().BoundingBox()
    print(f"Wrote {output}")
    print(
        f"Prop: {params.diameter / INCH:.2f} x {params.pitch / INCH:.2f} in, "
        f"{params.blade_count} blades, {params.direction}"
    )
    print(f"Size: X {bb.xlen:.2f}  Y {bb.ylen:.2f}  Z {bb.zlen:.2f} mm")


if __name__ == "__main__":
    main()
