"""ROS header.stamp to packet time. No rclpy import.

A missing header, a zero stamp, or a stamp in the future beyond the configured
tolerance is stale. A usable stamp is the source time, not the receipt time.
"""

from __future__ import annotations


def packet_time_from_header(message, now: float, future_tolerance_s: float):
    """Return (packet_time, stale_reason).

    packet_time is None when stale_reason is set. stale_reason is one of
    missing_stamp, zero_stamp, future_stamp.
    """
    header = getattr(message, "header", None)
    stamp = None if header is None else getattr(header, "stamp", None)
    if header is None or stamp is None:
        return None, "missing_stamp"
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None, "missing_stamp"
    if int(sec) == 0 and int(nanosec) == 0:
        return None, "zero_stamp"
    packet_time = float(sec) + float(nanosec) * 1e-9
    if packet_time > float(now) + float(future_tolerance_s):
        return None, "future_stamp"
    return packet_time, None


def stale_packet_time(core, now: float) -> float:
    """A production time the supervisor will treat as over-age."""
    return float(now) - core.freshness_threshold_s - 1.0


def stamped_packet_time(core, message, now: float, future_tolerance_s: float) -> tuple[float, str | None]:
    packet_time, reason = packet_time_from_header(message, now, future_tolerance_s)
    if packet_time is None:
        return stale_packet_time(core, now), reason
    return packet_time, None
