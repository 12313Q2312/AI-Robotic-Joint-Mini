"""Swappable intent source. v1 is a synthetic stair-ascent effort signal."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
import logging
import math
import random

from knee_stair_l1.topics import TOPIC_INTENT_SYNTHETIC

logger = logging.getLogger("knee_stair_l1.intent")


@dataclass(frozen=True)
class IntentSample:
    """One intent sample.

    effort_extension_nm is the assist-relevant extension request.
    human_extension_nm is what the human side of the plant applies.
    v1 sets those equal. source_label for the built-in source is synthetic.
    """

    source_label: str
    time_s: float
    effort_extension_nm: float
    ascent_active: bool
    phase: float
    human_extension_nm: float


class IntentSource(ABC):
    """Replacement point for a future estimator. v1 does not read EMG."""

    source_label: str

    @abstractmethod
    def reset(self, seed: int) -> None:
        raise NotImplementedError

    @abstractmethod
    def step(self, time_s: float) -> IntentSample:
        raise NotImplementedError


def amplitude_for_seed(seed: int, config: dict) -> float:
    """Deterministic amplitude shared by every condition of this seed."""
    intent = config["intent"]
    draw = random.Random(int(seed)).random()
    low = float(intent["amplitude_nm_min"])
    high = float(intent["amplitude_nm_max"])
    return low + draw * (high - low)


class SyntheticStairIntent(IntentSource):
    """Scripted single-burst knee extension effort.

    The log line, sample field and topic name all say synthetic.
    This is not EMG and not a recording from a person.
    """

    source_label = "synthetic"

    def __init__(self, config: dict):
        label = config["intent"].get("source_label")
        if label != "synthetic":
            raise ValueError('SyntheticStairIntent requires intent.source_label "synthetic"')
        self._config = config
        self.seed = 0
        self.amplitude_nm = 0.0

    def reset(self, seed: int) -> None:
        self.seed = int(seed)
        self.amplitude_nm = amplitude_for_seed(self.seed, self._config)
        logger.info(
            "synthetic intent source reset seed=%s source_label=synthetic topic=%s amplitude_nm=%.6f",
            self.seed,
            TOPIC_INTENT_SYNTHETIC,
            self.amplitude_nm,
        )

    def step(self, time_s: float) -> IntentSample:
        intent = self._config["intent"]
        onset = float(intent["onset_s"])
        duration = float(intent["burst_duration_s"])
        if time_s < onset or time_s >= onset + duration:
            effort = 0.0
            phase = 0.0
            active = False
        else:
            phase = (time_s - onset) / duration
            effort = self.amplitude_nm * math.sin(math.pi * phase) ** 2
            active = True
        return IntentSample(
            source_label="synthetic",
            time_s=float(time_s),
            effort_extension_nm=effort,
            ascent_active=active,
            phase=phase,
            human_extension_nm=effort,
        )
