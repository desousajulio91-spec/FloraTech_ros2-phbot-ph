"""ROS-independent tests for the synthetic soil field and electrode model."""

import math

import pytest

from phbot_ph.ph_simulation_core import SoilPhModel


@pytest.mark.parametrize('x,y,expected', [
    (-1.0, -1.0, 6.02), (0.0, 0.0, 6.38),
    (1.0, 1.0, 6.83), (5.0, 5.0, 7.43),
])
def test_ground_truth_passes_through_anchors(x, y, expected):
    assert SoilPhModel().true_ph_at(x, y) == pytest.approx(expected, abs=1e-5)


def test_negative_coordinates_are_measurable():
    model = SoilPhModel(seed=42)
    assert model.in_bounds(0.6, -0.3)
    result = model.measure(0.6, -0.3)
    assert math.isfinite(result.true_ph)
    assert math.isfinite(result.measured_ph)


@pytest.mark.parametrize('x,y', [(-1.01, 2.0), (5.01, 2.0),
                                 (1.0, -1.01), (1.0, 5.01),
                                 (math.nan, 1.0), (1.0, math.inf)])
def test_invalid_positions_rejected(x, y):
    model = SoilPhModel()
    assert not model.in_bounds(x, y)
    with pytest.raises(ValueError):
        model.measure(x, y)


def test_seed_reproducibly_generates_same_measurement():
    a = SoilPhModel(seed=123).measure(5.0, 5.0)
    b = SoilPhModel(seed=123).measure(5.0, 5.0)
    assert a == b
    assert 0.0 <= a.measured_ph <= 14.0


def test_zero_noise_has_only_adc_quantization_error():
    reading = SoilPhModel(noise_std_mv=0.0, adc_bits=16).measure(5.0, 5.0)
    assert abs(reading.measured_ph - reading.true_ph) < 0.001


def test_noise_changes_observations_but_not_ground_truth():
    model = SoilPhModel(noise_std_mv=5.0, seed=17)
    first = model.measure(4.0, 3.0)
    second = model.measure(4.0, 3.0)
    assert first.true_ph == pytest.approx(second.true_ph)
    assert first.measured_ph != second.measured_ph


def test_custom_field_region_uses_absolute_anchor_coordinates():
    model = SoilPhModel(width_m=3.0, height_m=2.0,
                        origin_x_m=0.0, origin_y_m=-1.0)
    assert model.in_bounds(0.6, -0.3)
    assert not model.in_bounds(-0.1, 0.0)
    # A smaller configured field must not stretch the anchor coordinates.
    assert model.true_ph_at(1.0, 0.0) == pytest.approx(
        SoilPhModel().true_ph_at(1.0, 0.0))


def test_field_corners_are_inside_without_nearest_neighbour_extrapolation():
    model = SoilPhModel()
    assert model.in_bounds(-1.0, -1.0)
    assert model.in_bounds(5.0, 5.0)
    assert model.true_ph_at(-1.0, -1.0) == pytest.approx(6.02, abs=1e-5)
    assert model.true_ph_at(5.0, 5.0) == pytest.approx(7.43, abs=1e-5)


@pytest.mark.parametrize('params', [
    {'width_m': 0.0}, {'height_m': math.inf},
    {'width_m': 6.1}, {'height_m': 6.1},
    {'origin_x_m': -1.1}, {'origin_y_m': -1.1},
    {'origin_x_m': 0.0}, {'origin_y_m': math.nan},
    {'noise_std_mv': -1}, {'adc_bits': 0}, {'adc_bits': 25},
])
def test_bad_model_configuration_rejected(params):
    with pytest.raises(ValueError):
        SoilPhModel(**params)
