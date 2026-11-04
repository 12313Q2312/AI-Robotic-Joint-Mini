"""Frictionless planar cam geometry in m, rad, N, J and N*m.

This first slice implements only the roller-follower geometry, not a wire cam,
optimization, contact dynamics, friction/hysteresis, actuator or CAD export.
No mass or gravitational load is silently assumed by the compensation model.
"""

from dataclasses import dataclass, field
import math
from numbers import Real


class GeometryError(ValueError):
    """Invalid or infeasible geometry; never substitute a desired torque curve."""


def _finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise GeometryError(f"{name} must be a finite real number")
    number = float(value)
    if not math.isfinite(number):
        raise GeometryError(f"{name} must be finite")
    return number


def _sample_count(value: int, name: str, minimum: int = 128) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= 65536:
        raise GeometryError(f"{name} must be an integer in [{minimum}, 65536]")
    return value


@dataclass(frozen=True)
class FourierCam:
    """r(phi) = mean + sum(a_n cos(n phi) + b_n sin(n phi)), n=1..4."""

    mean_radius_m: float
    cos_m: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    sin_m: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        radius = _finite(self.mean_radius_m, "mean_radius_m")
        if radius <= 0:
            raise GeometryError("mean_radius_m must be positive")
        object.__setattr__(self, "mean_radius_m", radius)
        for name in ("cos_m", "sin_m"):
            values = getattr(self, name)
            try:
                coefficients = tuple(values)
            except TypeError as exc:
                raise GeometryError(f"{name} must contain four coefficients") from exc
            if len(coefficients) != 4:
                raise GeometryError(f"{name} must contain exactly four coefficients")
            object.__setattr__(self, name,
                               tuple(_finite(v, name) for v in coefficients))

    def radius_derivatives(self, phi_rad: float) -> tuple[float, float, float]:
        """Return radius, dr/dphi, d2r/dphi2 in cam-local polar coordinates."""
        phi_rad = _finite(phi_rad, "phi_rad")
        radius, first, second = self.mean_radius_m, 0.0, 0.0
        for n, (a, b) in enumerate(zip(self.cos_m, self.sin_m), 1):
            c, s = math.cos(n * phi_rad), math.sin(n * phi_rad)
            term = a * c + b * s
            radius += term
            first += n * (-a * s + b * c)
            second -= n * n * term
        return radius, first, second

    def validate(self, min_radius_m: float = 0.005, max_radius_m: float = 0.040,
                 samples: int = 2048) -> dict[str, float]:
        """Conservatively certify whole-profile radius and strict convexity.

        Grid minima are tightened using global derivative/Lipschitz bounds, so
        a narrow invalid interval between nodes is not silently accepted. A
        rejected borderline profile may become certifiable on a finer grid.
        """
        lower = _finite(min_radius_m, "min_radius_m")
        upper = _finite(max_radius_m, "max_radius_m")
        samples = _sample_count(samples, "samples")
        if not 0 < lower < upper:
            raise GeometryError("Require 0 < min_radius_m < max_radius_m")
        amplitudes = [math.hypot(a, b) for a, b in zip(self.cos_m, self.sin_m)]
        bounds = [sum((n**order) * a for n, a in enumerate(amplitudes, 1))
                  for order in range(4)]
        r_bound = self.mean_radius_m + bounds[0]
        if not all(math.isfinite(v) for v in (*bounds, r_bound)):
            raise GeometryError("Profile coefficient magnitude overflows")
        values = [self.radius_derivatives(math.tau * i / samples)
                  for i in range(samples)]
        half_step = math.pi / samples
        radius_min = max(self.mean_radius_m - bounds[0],
                         min(r for r, _, _ in values) - bounds[1] * half_step)
        radius_max = min(r_bound,
                         max(r for r, _, _ in values) + bounds[1] * half_step)
        if radius_min < lower - 1e-12 or radius_max > upper + 1e-12:
            raise GeometryError("Whole cam profile violates radius limits")
        numerator_min = min(r*r + 2*rp*rp - r*rpp for r, rp, rpp in values)
        lipschitz = 2*r_bound*bounds[1] + 3*bounds[1]*bounds[2] + r_bound*bounds[3]
        convexity_margin = numerator_min - lipschitz * half_step
        if not math.isfinite(convexity_margin) or convexity_margin <= 0:
            raise GeometryError("Strict convexity is not certified over the whole cam")
        return {"min_radius_bound_m": radius_min,
                "max_radius_bound_m": radius_max,
                "convexity_margin_m2": convexity_margin}


@dataclass(frozen=True)
class CompensationState:
    """One frictionless contact solution, not measured actuator torque."""

    q_rad: float
    contact_phi_rad: float
    follower_y_m: float
    displacement_m: float
    spring_compression_m: float
    spring_force_n: float
    energy_j: float
    torque_nm: float
    pressure_angle_rad: float


@dataclass(frozen=True)
class RollerCompensator:
    """Upper roller follower on fixed world x=offset, compressed upward.

    The cam rotates counterclockwise by q about world z. The follower moves
    along +y; preload_m is initial spring *compression*, not preload force.
    """

    cam: FourierCam
    roller_radius_m: float
    follower_offset_m: float
    spring_stiffness_n_per_m: float
    preload_m: float
    max_compression_m: float
    contact_samples: int = 512
    angle_limit_rad: float = math.pi / 4
    max_envelope_radius_m: float = 0.040
    _profile: tuple = field(init=False, repr=False, compare=False)
    _reference_y_m: float = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if not isinstance(self.cam, FourierCam):
            raise GeometryError("cam must be a FourierCam")
        for name in ("roller_radius_m", "follower_offset_m", "spring_stiffness_n_per_m",
                     "preload_m", "max_compression_m", "angle_limit_rad",
                     "max_envelope_radius_m"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        _sample_count(self.contact_samples, "contact_samples")
        if self.roller_radius_m <= 0 or self.spring_stiffness_n_per_m <= 0:
            raise GeometryError("Roller radius and spring stiffness must be positive")
        if not 0 < self.preload_m < self.max_compression_m:
            raise GeometryError("Require 0 < preload_m < max_compression_m")
        if not 0 < self.angle_limit_rad <= math.pi:
            raise GeometryError("angle_limit_rad must be in (0, pi]")
        # Conservative radial bound covers both cam and entire roller, not just
        # the roller centre. It does not certify housings, spring or base.
        self.cam.validate(max_radius_m=self.max_envelope_radius_m - 2*self.roller_radius_m)
        profile = tuple(self._offset_point(math.tau * i / self.contact_samples)
                        for i in range(self.contact_samples + 1))
        object.__setattr__(self, "_profile", profile)
        object.__setattr__(self, "_reference_y_m", self._contact(0.0)[1])

    def _offset_point(self, phi: float) -> tuple[float, ...]:
        r, rp, rpp = self.cam.radius_derivatives(phi)
        c, s = math.cos(phi), math.sin(phi)
        speed = math.hypot(r, rp)
        nx, ny = (r * c + rp * s) / speed, (r * s - rp * c) / speed
        curvature = (r * r + 2 * rp * rp - r * rpp) / speed**3
        scale = 1.0 + self.roller_radius_m * curvature
        return (r * c + self.roller_radius_m * nx,
                r * s + self.roller_radius_m * ny,
                scale * (rp * c - r * s),
                scale * (rp * s + r * c), nx, ny)

    def _contact(self, q: float) -> tuple[float, float, float, float]:
        c, s = math.cos(q), math.sin(q)
        if not any(self.cam.cos_m) and not any(self.cam.sin_m):
            radius = self.cam.mean_radius_m + self.roller_radius_m
            if abs(self.follower_offset_m) >= radius:
                raise GeometryError("No nonsingular upper follower contact")
            height = math.sqrt((radius - self.follower_offset_m)
                               * (radius + self.follower_offset_m))
            if height / radius < 1e-6:
                raise GeometryError("Follower contact is near a singular pressure angle")
            alpha = math.atan2(height, self.follower_offset_m)
            return ((alpha - q) % math.tau, height, 0.0,
                    math.atan2(self.follower_offset_m, height))

        def residual(phi: float) -> float:
            x, y, *_ = self._offset_point(phi)
            return c * x - s * y - self.follower_offset_m

        roots = []
        step = math.tau / self.contact_samples
        residuals = [c * p[0] - s * p[1] - self.follower_offset_m
                     for p in self._profile]
        for i in range(self.contact_samples):
            left, right = i * step, (i + 1) * step
            fl, fr = residuals[i], residuals[i + 1]
            if abs(fl) < 1e-14:
                roots.append(left % math.tau)
            if fl * fr >= 0:
                continue
            for _ in range(60):
                middle = 0.5 * (left + right)
                fm = residual(middle)
                if abs(fm) < 1e-14 or right - left < 1e-13:
                    left = right = middle
                    break
                if fl * fm <= 0:
                    right = middle
                else:
                    left, fl = middle, fm
            roots.append((0.5 * (left + right)) % math.tau)
        contacts = []
        for phi in roots:
            if any(abs(math.remainder(phi - old[0], math.tau)) < 1e-8
                   for old in contacts):
                continue
            x, y, xp, yp, nx, ny = self._offset_point(phi)
            normal_y = nx * s + ny * c
            if normal_y <= 0:
                continue  # The lower boundary cannot support this upper follower.
            world_x, world_y = x * c - y * s, x * s + y * c
            world_xp, world_yp = xp * c - yp * s, xp * s + yp * c
            if abs(world_xp) < 1e-10 or normal_y < 1e-6:
                raise GeometryError("Follower contact is near a tangency/singular pressure angle")
            # At fixed X=offset: dphi/dq=Y/(dX/dphi), since dX/dq=-Y.
            # Then dY/dq=X+(dY/dphi)*(dphi/dq). This implicit derivative
            # is independent of the spring energy difference used by tests.
            slope = world_x + world_yp * world_y / world_xp
            pressure = math.atan2(nx * c - ny * s, normal_y)
            contacts.append((phi, world_y, slope, pressure))
        if len(contacts) != 1:
            raise GeometryError("Expected exactly one upper outward contact branch")
        return contacts[0]

    def evaluate(self, q_rad: float) -> CompensationState:
        """Return real geometry, spring energy and passive torque at q."""
        q_rad = _finite(q_rad, "q_rad")
        if abs(q_rad) > self.angle_limit_rad + 1e-12:
            raise GeometryError("Angle is outside the configured mechanical range")
        phi, height, slope, pressure = self._contact(q_rad)
        displacement = height - self._reference_y_m
        compression = self.preload_m + displacement
        if not 0 < compression <= self.max_compression_m:
            raise GeometryError("Spring unloads or exceeds its compression limit")
        force = self.spring_stiffness_n_per_m * compression
        energy = 0.5 * self.spring_stiffness_n_per_m * compression**2
        if not all(math.isfinite(v) for v in (force, energy, slope, -force*slope)):
            raise GeometryError("Computed spring state is not finite")
        return CompensationState(
            q_rad, phi, height, displacement,
            compression, force, energy, -force * slope, pressure,
        )

    def validate_range(self, q_min_rad: float | None = None,
                       q_max_rad: float | None = None,
                       samples: int = 181) -> dict[str, float | int]:
        """Check every requested angle-grid node; fail rather than clip.

        Radius/convexity are whole-profile certificates. This range report is
        explicitly a sampled contact/spring audit, NOT proof between nodes,
        contact stress analysis, or manufacturing/physical validation.
        """
        lower = -self.angle_limit_rad if q_min_rad is None else _finite(q_min_rad, "q_min_rad")
        upper = self.angle_limit_rad if q_max_rad is None else _finite(q_max_rad, "q_max_rad")
        _sample_count(samples, "samples", minimum=3)
        if not -self.angle_limit_rad <= lower < upper <= self.angle_limit_rad:
            raise GeometryError("Audit range must be ordered and within mechanical limits")
        states = [self.evaluate(lower + (upper - lower) * i / (samples - 1))
                  for i in range(samples)]
        return {"q_min_rad": lower, "q_max_rad": upper, "samples": samples,
                "min_compression_m": min(s.spring_compression_m for s in states),
                "max_compression_m": max(s.spring_compression_m for s in states),
                "min_torque_nm": min(s.torque_nm for s in states),
                "max_torque_nm": max(s.torque_nm for s in states),
                "max_abs_pressure_angle_rad": max(abs(s.pressure_angle_rad) for s in states)}
