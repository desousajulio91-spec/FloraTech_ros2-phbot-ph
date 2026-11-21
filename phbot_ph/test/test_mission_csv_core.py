"""Tests for mission-specific pH CSV naming and existing schema preservation."""
import csv
import re

import pytest

from phbot_ph.mission_csv_core import MissionCsvRecorder, output_path_for_mission


FIELDS = ('timestamp_utc', 'sample_id', 'x_m', 'y_m',
          'target_x_m', 'target_y_m', 'target_error_m',
          'true_ph', 'measured_ph', 'temperature_c',
          'electrode_mv', 'adc_voltage')


def record(site_id):
    return dict(timestamp_utc='2026-11-10T18:00:00+00:00',
                sample_id=site_id,
                x_m=0.34, y_m=-0.28,
                target_x_m=0.30, target_y_m=-0.30,
                target_error_m=0.045,
                true_ph=6.5, measured_ph=6.48,
                temperature_c=20.1, electrode_mv=30.0,
                adc_voltage=2.53)


def test_same_mission_appends_and_preserves_all_columns(tmp_path):
    base = tmp_path / 'telemetry.csv'
    rec = MissionCsvRecorder(base, FIELDS)
    p0 = rec.write('20261110T182233Z_abc', record(2))
    p1 = rec.write('20261110T182233Z_abc', record(4))
    assert p0 == p1
    assert not base.exists()
    with p0.open(newline='') as f:
        reader = csv.DictReader(f)
        assert reader.fieldnames == list(FIELDS)
        rows = list(reader)
    assert len(rows) == 2
    assert [int(r['sample_id']) for r in rows] == [2, 4]
    assert rows[0]['x_m'] == '0.34'
    assert rows[0]['target_x_m'] == '0.3'
    assert rows[0]['measured_ph'] == '6.48'


def test_different_missions_have_different_files(tmp_path):
    rec = MissionCsvRecorder(tmp_path / 'telemetry.csv', FIELDS)
    a = rec.write('mission_A', record(0))
    b = rec.write('mission_B', record(0))
    assert a != b
    assert a.exists() and b.exists()


def test_restart_reuses_mission_file(tmp_path):
    base = tmp_path / 'telemetry.csv'
    first = MissionCsvRecorder(base, FIELDS)
    first.write('same_mission', record(0))
    second = MissionCsvRecorder(base, FIELDS)
    p = second.write('same_mission', record(1))
    with p.open(newline='') as f:
        assert len(list(csv.DictReader(f))) == 2


def test_standalone_goals_do_not_share_a_file(tmp_path):
    rec = MissionCsvRecorder(tmp_path / 'telemetry.csv', FIELDS)
    a = rec.write('', record(0))
    b = rec.write('', record(1))
    assert a != b
    assert re.search(r'_standalone_[0-9a-f]{32}\.csv$', a.name)


def test_incompatible_header_is_rejected_without_overwrite(tmp_path):
    base = tmp_path / 'telemetry.csv'
    path = output_path_for_mission(base, 'sample')
    path.write_text('old_column\nold_data\n')
    rec = MissionCsvRecorder(base, FIELDS)
    with pytest.raises(ValueError, match='existing_mission_csv_schema_mismatch'):
        rec.write('sample', record(1))
    assert path.read_text() == 'old_column\nold_data\n'


@pytest.mark.parametrize('bad_id', [
    '../escape', 'a/b', r'a\\b', '.hidden', 'space name',
    'a' * 97, 'é', 'abc.csv',
])
def test_invalid_ids_rejected(tmp_path, bad_id):
    with pytest.raises(ValueError, match='invalid_mission_id'):
        output_path_for_mission(tmp_path / 'telemetry.csv', bad_id)


def test_base_csv_is_never_modified(tmp_path):
    base = tmp_path / 'telemetry.csv'
    base.write_text('historical,record\n1,2\n')
    p = MissionCsvRecorder(base, FIELDS).write('first', record(0))
    assert p != base
    assert base.read_text() == 'historical,record\n1,2\n'


def test_requires_csv_extension(tmp_path):
    with pytest.raises(ValueError, match='csv_path_must_have_csv_extension'):
        MissionCsvRecorder(tmp_path / 'telemetry.txt', FIELDS)
