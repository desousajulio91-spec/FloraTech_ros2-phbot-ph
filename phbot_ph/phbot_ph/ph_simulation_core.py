"""ROS-independent synthetic soil pH field and idealized electrode simulation.

The dense anchor values are illustrative, not surveyed soil measurements.
Their coordinates use the robot's local map frame. The synthetic field is
ground truth for simulation only; mapping must use measured samples.
"""

import math
import random
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CloughTocher2DInterpolator, NearestNDInterpolator


# Ground-truth anchor coordinates
# 0.5 m spacing across a 6 x 6 m field.

_anchor_x = np.arange(-1.0, 5.01, 0.5)
_anchor_y = np.arange(-1.0, 5.01, 0.5)

_X, _Y = np.meshgrid(_anchor_x, _anchor_y)

_ANCHORS = np.column_stack((_X.ravel(), _Y.ravel()))


# Corresponding ground-truth pH values.
# Rows correspond to increasing Y coordinates.
# Columns correspond to increasing X coordinates.

_ANCHOR_PH = np.array([
    [6.02, 6.16, 6.31, 6.46, 6.59, 6.70, 6.77, 6.82, 6.82, 6.81, 6.78, 6.75, 6.73],
    [6.02, 6.18, 6.35, 6.52, 6.67, 6.79, 6.87, 6.90, 6.90, 6.86, 6.81, 6.76, 6.72],
    [6.02, 6.20, 6.38, 6.57, 6.73, 6.86, 6.95, 6.98, 6.97, 6.92, 6.85, 6.78, 6.73],
    [6.05, 6.23, 6.42, 6.61, 6.79, 6.93, 7.03, 7.06, 7.04, 6.97, 6.89, 6.81, 6.75],
    [6.09, 6.26, 6.45, 6.64, 6.83, 7.00, 7.11, 7.15, 7.11, 7.03, 6.93, 6.85, 6.79],
    [6.14, 6.30, 6.47, 6.66, 6.86, 7.05, 7.19, 7.23, 7.18, 7.08, 6.98, 6.90, 6.85],
    [6.20, 6.32, 6.46, 6.64, 6.85, 7.06, 7.22, 7.27, 7.22, 7.12, 7.02, 6.95, 6.92],
    [6.26, 6.32, 6.40, 6.55, 6.78, 7.00, 7.16, 7.22, 7.18, 7.11, 7.04, 7.01, 7.01],
    [6.33, 6.33, 6.34, 6.46, 6.67, 6.88, 7.02, 7.09, 7.09, 7.07, 7.05, 7.06, 7.10],
    [6.42, 6.39, 6.37, 6.44, 6.59, 6.75, 6.87, 6.94, 6.98, 7.02, 7.06, 7.12, 7.19],
    [6.52, 6.50, 6.47, 6.49, 6.57, 6.66, 6.74, 6.82, 6.90, 6.98, 7.07, 7.18, 7.28],
    [6.62, 6.59, 6.56, 6.55, 6.57, 6.60, 6.66, 6.74, 6.84, 6.96, 7.09, 7.23, 7.36],
    [6.70, 6.66, 6.61, 6.58, 6.56, 6.57, 6.61, 6.70, 6.81, 6.96, 7.12, 7.28, 7.43],
], dtype=float).ravel()

# Coordinates are physical metres in the local map frame; do not normalize
# them into the former 0..10 m demonstration field.
_FIELD_MIN_X = float(_anchor_x[0])
_FIELD_MAX_X = float(_anchor_x[-1])
_FIELD_MIN_Y = float(_anchor_y[0])
_FIELD_MAX_Y = float(_anchor_y[-1])


@dataclass(frozen=True)
class PhReading:
    """Synthetic diagnostic output from one sampling event."""

    true_ph: float
    measured_ph: float
    temperature_c: float
    electrode_mv: float
    adc_voltage: float


class SoilPhModel:
    """Synthetic spatial truth and noisy, quantized pH measurement model."""

    def __init__(self, width_m=6.0, height_m=6.0, noise_std_mv=2.0,
                 adc_bits=12, seed=42, origin_x_m=-1.0, origin_y_m=-1.0):
        if not all(math.isfinite(v) and v > 0 for v in (width_m, height_m)):
            raise ValueError('field dimensions must be positive and finite')
        if not all(math.isfinite(v) for v in (origin_x_m, origin_y_m)):
            raise ValueError('field origin must be finite')
        # These anchors cover [-1, 5] metres along both axes. A smaller map
        # may occupy a subset, but a map beyond the anchors is not fabricated
        # using nearest-neighbour extrapolation.
        if (origin_x_m < _FIELD_MIN_X or origin_y_m < _FIELD_MIN_Y
                or origin_x_m + width_m > _FIELD_MAX_X
                or origin_y_m + height_m > _FIELD_MAX_Y):
            raise ValueError('configured field exceeds ground-truth anchor bounds')
        if not math.isfinite(noise_std_mv) or noise_std_mv < 0:
            raise ValueError('noise_std_mv must be finite and nonnegative')
        if not 1 <= adc_bits <= 24:
            raise ValueError('adc_bits must be between 1 and 24')

        self.origin_x_m = float(origin_x_m)
        self.origin_y_m = float(origin_y_m)
        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.noise_std_mv = float(noise_std_mv)
        self.adc_bits = int(adc_bits)
        self._rng = random.Random(seed)
        self._cubic = CloughTocher2DInterpolator(_ANCHORS, _ANCHOR_PH)
        self._nearest = NearestNDInterpolator(_ANCHORS, _ANCHOR_PH)

    def in_bounds(self, x, y):
        """Check absolute map-frame coordinates against the configured region."""
        return (math.isfinite(x) and math.isfinite(y)
                and self.origin_x_m <= x <= self.origin_x_m + self.width_m
                and self.origin_y_m <= y <= self.origin_y_m + self.height_m)

    def true_ph_at(self, x, y):
        """Interpolate the pH field at physical map coordinates in metres."""
        if not self.in_bounds(x, y):
            raise ValueError('sampling coordinates are outside the synthetic field')
        # Direct map-frame evaluation: anchors are already defined in metres.
        ph = float(self._cubic(x, y))
        if not math.isfinite(ph):
            ph = float(self._nearest(x, y))
        return min(14.0, max(0.0, ph))

    def measure(self, x, y):
        """Apply the Nernst slope, electrode noise and an illustrative ADC model.

        The 0--5 V ADC model is virtual; it is NOT safe to wire 5 V to an ESP32.
        The dwell time belongs to the ROS action, not the electrochemical model.
        """
        true_ph = self.true_ph_at(x, y)
        temperature_c = self._rng.uniform(19.0, 23.5)
        temperature_k = temperature_c + 273.15
        slope_mv_per_ph = 1000.0 * (2.303 * 8.314462618 * temperature_k / 96485.33212)

        ideal_mv = (7.0 - true_ph) * slope_mv_per_ph
        electrode_mv = ideal_mv + self._rng.gauss(0.0, self.noise_std_mv)
        analog_v = 2.5 + electrode_mv / 1000.0

        # Idealized 5 V unipolar, finite-resolution ADC, as in the team model.
        if not 0.0 <= analog_v <= 5.0:
            raise ValueError('simulated electrode voltage outside ADC range')
        levels = (1 << self.adc_bits) - 1
        counts = round(analog_v * levels / 5.0)
        adc_voltage = counts * 5.0 / levels
        measured_ph = 7.0 - ((adc_voltage - 2.5) * 1000.0 / slope_mv_per_ph)
        return PhReading(true_ph, measured_ph, temperature_c,
                         electrode_mv, adc_voltage)
