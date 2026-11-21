"""Per-mission pH CSV persistence, independent of ROS.

The configured csv_path is a filename *template*, never overwritten:
  /dir/rover_ph_pose_telemetry_log.csv
becomes:
  /dir/rover_ph_pose_telemetry_log_<mission-id>.csv

All rows from one mission share the same path. Empty mission IDs (manual
action requests) receive a unique per-sample path to avoid accidental merging.
"""
import csv
import re
from pathlib import Path
from uuid import uuid4


_ID_PATTERN = re.compile(r'^[A-Za-z0-9_-]{1,96}$')


def output_path_for_mission(csv_path, mission_id):
    """Build a safe path without allowing path traversal or overwriting base."""
    base = Path(csv_path).expanduser()
    if not base.suffix.lower() == '.csv':
        raise ValueError('csv_path_must_have_csv_extension')
    if mission_id:
        if not _ID_PATTERN.fullmatch(mission_id):
            raise ValueError('invalid_mission_id')
        identifier = mission_id
    else:
        identifier = 'standalone_' + uuid4().hex
    return base.with_name(f'{base.stem}_{identifier}{base.suffix}')


class MissionCsvRecorder:
    """Preserve the existing diagnostic CSV schema, grouped by mission ID."""

    def __init__(self, csv_path, fields):
        self.base_path = Path(csv_path).expanduser()
        self.fields = tuple(fields)
        # Path resolution is deterministic for a given nonempty mission ID.
        output_path_for_mission(self.base_path, 'validation')

    def write(self, mission_id, row):
        path = output_path_for_mission(self.base_path, mission_id)
        path.parent.mkdir(parents=True, exist_ok=True)

        # Validate before touching an existing file. This also ensures a
        # restarted action server can append to the same mission file.
        if path.exists() and path.stat().st_size:
            with path.open('r', newline='', encoding='utf-8') as stream:
                header = next(csv.reader(stream), None)
            if header != list(self.fields):
                raise ValueError('existing_mission_csv_schema_mismatch')
        with path.open('a', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=self.fields)
            if stream.tell() == 0:
                writer.writeheader()
            writer.writerow(row)
        return path
