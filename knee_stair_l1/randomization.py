"""Paired per-seed draws. The same seed produces the same factors for every condition.

Streams are separate so measurement noise and delay jitter do not change the
factor draws. random.Random is seeded explicitly. Gaussian noise uses
Box-Muller on Random.random so the sequence does not depend on Random.gauss.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class SeedFactors:
    seed: int
    body_mass_kg: float
    step_height_m: float
    intent_amplitude_scale: float
    intent_onset_s: float
    strap_stiffness_scale: float

    def as_dict(self) -> dict:
        return {
            "seed": self.seed,
            "body_mass_kg": self.body_mass_kg,
            "step_height_m": self.step_height_m,
            "intent_amplitude_scale": self.intent_amplitude_scale,
            "intent_onset_s": self.intent_onset_s,
            "strap_stiffness_scale": self.strap_stiffness_scale,
        }


def draw_factors(seed: int, config: dict) -> SeedFactors:
    span = config["randomization"]
    nominal_onset = float(config["intent"]["onset_s"])
    rng = random.Random(int(seed))
    mass = rng.uniform(float(span["body_mass_kg_min"]), float(span["body_mass_kg_max"]))
    height = rng.uniform(float(span["step_height_m_min"]), float(span["step_height_m_max"]))
    amp_frac = float(span["intent_amplitude_fraction"])
    amplitude = rng.uniform(1.0 - amp_frac, 1.0 + amp_frac)
    onset_jitter = float(span["intent_onset_jitter_s"])
    onset = nominal_onset + rng.uniform(-onset_jitter, onset_jitter)
    strap_frac = float(span["strap_stiffness_fraction"])
    strap_scale = rng.uniform(1.0 - strap_frac, 1.0 + strap_frac)
    return SeedFactors(
        seed=int(seed),
        body_mass_kg=mass,
        step_height_m=height,
        intent_amplitude_scale=amplitude,
        intent_onset_s=onset,
        strap_stiffness_scale=strap_scale,
    )


def gaussian(rng: random.Random) -> float:
    """Standard normal from two Random.random draws."""
    u1 = max(rng.random(), 1e-12)
    u2 = rng.random()
    return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)


def noise_rng(seed: int) -> random.Random:
    return random.Random(100_003 + int(seed))


def jitter_rng(seed: int) -> random.Random:
    return random.Random(200_003 + int(seed))
