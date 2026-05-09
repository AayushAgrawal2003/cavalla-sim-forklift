"""Stop-condition deciders for the drop-off line follower.

The follower controller calls `decider.should_stop(line_state, ...)` once
per tick. Today the only implementation is `LineStopDecider`, which
returns True when the perpendicular yellow stop line is detected for at
least `min_consec_frames` consecutive *unique* line_state frames
(optionally restricted to a horizontal band). When we later swap to
LiDAR-based proximity sensing (the user mentioned this is on the
roadmap), we add a new `LidarStopDecider` here that consumes
`/scan` -- the controller's call site stays unchanged.

Decoupling the stop decision from the control loop keeps the swap
to a few lines of wiring and lets us A/B test perception-based vs
range-based stopping cleanly.

Why "consecutive UNIQUE frames" matters: the controller ticks at 50 Hz
but line_state arrives at ~18-20 Hz, so the same line_state is reused
across 2-3 ticks. We dedupe by `header.stamp` so the consecutive count
reflects camera frames, not control ticks.
"""

from __future__ import annotations

from typing import Optional, Protocol

from drop_off_msgs.msg import LineState


class StopDecider(Protocol):
    """Protocol that all stop deciders must satisfy."""

    def should_stop(
        self,
        line_state: Optional[LineState],
    ) -> tuple[bool, str]:
        """Return (should_stop, reason)."""
        ...


class LineStopDecider:
    """Decide based on the perpendicular yellow stop line.

    Triggers `True` when the stop line is detected for `min_consec_frames`
    consecutive line_state frames (deduplicated by header stamp) AND its
    centroid sits inside an optional horizontal band of the image. The
    band is expressed in fractions of image height measured from the
    bottom; defaults to the whole image so any in-frame stop counts.

    The decider keeps internal state (last-seen frame stamp, consecutive
    count). All thresholds are passed per-call so the controller can
    drive them from ROS parameters and `ros2 param set` takes effect on
    the next tick without rebuilding.
    """

    def __init__(
        self,
        detected_attr: str = 'stop_detected',
        distance_attr: str = 'stop_distance_px',
    ) -> None:
        self._last_stamp: Optional[tuple[int, int]] = None
        self._consec: int = 0
        # Field names to read off LineState. Defaults match the primary
        # (red) stop tape; the back_out path constructs a second decider
        # pointed at the yellow_stop_* fields.
        self._detected_attr = detected_attr
        self._distance_attr = distance_attr

    def reset(self) -> None:
        """Drop any in-flight consecutive count and frame memory.

        Called by the controller on mode-entry and on transitions out
        of an active mode, so a stale count from a previous run can't
        carry over and trigger a spurious immediate stop.
        """
        self._last_stamp = None
        self._consec = 0

    def should_stop(
        self,
        line_state: Optional[LineState],
        *,
        stop_band_low_frac: float = 0.0,
        stop_band_high_frac: float = 1.0,
        min_consec_frames: int = 5,
    ) -> tuple[bool, str]:
        min_consec = max(1, int(min_consec_frames))
        if line_state is None:
            return False, ''

        stamp = (
            int(line_state.header.stamp.sec),
            int(line_state.header.stamp.nanosec),
        )
        new_frame = stamp != self._last_stamp

        # Determine whether THIS frame would count as a stop sample.
        in_band = True
        h = int(line_state.image_height)
        if h > 0 and (
            stop_band_low_frac > 1e-6 or stop_band_high_frac < 1.0 - 1e-6
        ):
            low = stop_band_low_frac * h
            high = stop_band_high_frac * h
            d = float(getattr(line_state, self._distance_attr))
            in_band = low <= d <= high

        sample = bool(getattr(line_state, self._detected_attr)) and in_band

        if new_frame:
            self._last_stamp = stamp
            if sample:
                self._consec += 1
            else:
                self._consec = 0

        if self._consec < min_consec:
            if sample:
                return False, (
                    f'stop pending ({self._consec}/{min_consec})'
                )
            return False, ''

        return True, (
            f'stop confirmed ({self._consec} consec frames)'
        )
