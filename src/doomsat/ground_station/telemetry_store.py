"""In-memory circular buffer and time-series store for DOOMSat telemetry beacons."""

from collections import deque
from typing import Deque, Dict, List, Optional
import json
import io
import csv

from doomsat.core.schemas import TelemetryBeacon


class TelemetryStore:
    """Manages received telemetry beacons with circular buffering and querying."""

    def __init__(self, max_records: int = 10_000) -> None:
        self.max_records = max_records
        self._beacons: Deque[TelemetryBeacon] = deque(maxlen=max_records)

    def add_beacon(self, beacon: TelemetryBeacon) -> None:
        """Append a newly received and validated TelemetryBeacon."""
        self._beacons.append(beacon)

    def get_latest(self) -> Optional[TelemetryBeacon]:
        """Return the most recently received TelemetryBeacon, or None if empty."""
        if not self._beacons:
            return None
        return self._beacons[-1]

    def get_count(self) -> int:
        """Return total number of beacons currently stored."""
        return len(self._beacons)

    def get_history(
        self,
        window_seconds: Optional[float] = None,
        max_records: int = 500,
    ) -> List[TelemetryBeacon]:
        """Query historical beacons, optionally filtered by time window.

        Args:
            window_seconds: If provided, returns beacons within [latest_time - window_seconds, latest_time].
            max_records: Maximum number of records to return.

        Returns:
            List of TelemetryBeacon objects ordered chronologically.
        """
        if not self._beacons:
            return []

        beacons_list = list(self._beacons)
        if window_seconds is not None and len(beacons_list) > 0:
            latest_ts = beacons_list[-1].timestamp_s
            threshold_ts = latest_ts - window_seconds
            beacons_list = [b for b in beacons_list if b.timestamp_s >= threshold_ts]

        if len(beacons_list) > max_records:
            beacons_list = beacons_list[-max_records:]

        return beacons_list

    def export_json(self, window_seconds: Optional[float] = None) -> str:
        """Export stored beacons to JSON formatted string."""
        beacons = self.get_history(window_seconds=window_seconds, max_records=self.max_records)
        return json.dumps([b.model_dump() for b in beacons], indent=2)

    def export_csv(self, window_seconds: Optional[float] = None) -> str:
        """Export stored beacons to CSV formatted string."""
        beacons = self.get_history(window_seconds=window_seconds, max_records=self.max_records)
        if not beacons:
            return ""

        output = io.StringIO()
        fieldnames = list(beacons[0].model_dump().keys())
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        for b in beacons:
            writer.writerow(b.model_dump())
        return output.getvalue()

    def clear(self) -> None:
        """Clear all stored beacons."""
        self._beacons.clear()
