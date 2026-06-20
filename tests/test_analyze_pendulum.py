import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from analyze_pendulum import fit_motion


def test_fit_motion_recovers_frequency_and_amplitude():
    t = np.linspace(0.0, 12.0, 360)
    x = 112.0 + 36.0 * np.cos(2.0 * math.pi * 0.72 * t + 0.4)

    result = fit_motion(t, x)

    assert abs(result.frequency_hz - 0.72) < 0.01
    assert abs(result.amplitude_px - 36.0) < 0.5
    assert abs(result.offset_px - 112.0) < 0.5
