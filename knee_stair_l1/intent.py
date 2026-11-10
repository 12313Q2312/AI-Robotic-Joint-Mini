"""Swappable intent source.

v1 is a synthetic desired knee trajectory. The human torque that tracks it is
bounded separately. Logs, the topic name and source_label say synthetic.
This is not EMG and not a recording from a person.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
import logging
import math

from knee_stair_l1.randomization import draw_factors
from knee_stair_l1.topics import TOPIC_INTENT_SYNTHETIC

logger = logging.getLogger("knee_stair_l1.intent")


@dataclass(frozen=True)
class IntentSample:
    source_label: str
    time_s: float
    q_ref_rad: float
    qd_ref_rad_s: float
    ascent_active: bool
    phase: float


class IntentSource(ABC):
    source_label: str

    @abstractmethod
    def reset(self, seed: int) -> None:
        raise NotImplementedError

    @abstractmethod
    def step(self, time_s: float) -> IntentSample:
        raise NotImplementedError


def depth_scale_for_seed(seed: int, config: dict) -> float:
    """Paired amplitude scale. Every condition of this seed uses the same draw."""
    return draw_factors(seed, config).intent_amplitude_scale


def _smoothstep(phase: float) -> tuple[float, float]:
    s = phase * phase * (3.0 - 2.0 * phase)
    ds_dphase = 6.0 * phase * (1.0 - phase)
    return s, ds_dphase


class SyntheticStairIntent(IntentSource):
    """Smooth flexion-to-less-flexion trajectory. Depth varies by seed and stays bounded."""

    source_label = "synthetic"

    def __init__(self, config: dict):
        if config["intent"].get("source_label") != "synthetic":
            raise ValueError('SyntheticStairIntent requires intent.source_label "synthetic"')
        self._config = config
        self.seed = 0
        self.depth_scale = 1.0
        self.q_start = float(config["intent"]["q_start_rad"])
        self.q_end = float(config["intent"]["q_end_rad"])

    def reset(self, seed: int) -> None:
        self.seed = int(seed)
        self.factors = draw_factors(self.seed, self._config)
        self.depth_scale = self.factors.intent_amplitude_scale
        self.onset_s = self.factors.intent_onset_s
        logger.info(
            "synthetic intent source reset seed=%s source_label=synthetic topic=%s depth_scale=%.6f",
            self.seed,
            TOPIC_INTENT_SYNTHETIC,
            self.depth_scale,
        )

    def q_target(self) -> float:
        span = self.q_start - self.q_end
        return self.q_start - span * self.depth_scale

    def step(self, time_s: float) -> IntentSample:
        intent = self._config["intent"]
        onset = float(self.onset_s)
        duration = float(intent["move_duration_s"])
        target = self.q_target()
        if time_s <= onset:
            q_ref, qd_ref, phase, active = self.q_start, 0.0, 0.0, False
        elif time_s >= onset + duration:
            q_ref, qd_ref, phase, active = target, 0.0, 1.0, False
        else:
            phase = (time_s - onset) / duration
            shaped, slope = _smoothstep(phase)
            q_ref = self.q_start + (target - self.q_start) * shaped
            qd_ref = (target - self.q_start) * slope / duration
            active = True
        if not math.isfinite(q_ref):
            raise ValueError("synthetic reference is not finite")
        return IntentSample(
            source_label="synthetic",
            time_s=float(time_s),
            q_ref_rad=q_ref,
            qd_ref_rad_s=qd_ref,
            ascent_active=active,
            phase=phase,
        )
