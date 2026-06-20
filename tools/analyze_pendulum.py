#!/usr/bin/env python3
"""Track a pendulum bob in video and fit simple harmonic motion.

The fitted model is:
    x(t) = x0 + A cos(2*pi*f*t + phi)

For small-angle pendulum motion:
    T = 1 / f
    g = (2*pi*f)^2 * L
where L is the physical pendulum length in meters.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


HSV_PRESETS = {
    "red": ((0, 80, 50), (10, 255, 255), (170, 80, 50), (180, 255, 255)),
    "green": ((35, 50, 40), (90, 255, 255)),
    "blue": ((90, 50, 40), (135, 255, 255)),
    "bright": ((0, 0, 170), (180, 80, 255)),
    "dark": ((0, 0, 0), (180, 255, 80)),
}


@dataclass(frozen=True)
class FitResult:
    offset_px: float
    amplitude_px: float
    frequency_hz: float
    phase_rad: float
    rmse_px: float

    @property
    def omega(self) -> float:
        return 2.0 * math.pi * self.frequency_hz

    @property
    def period_s(self) -> float:
        return 1.0 / self.frequency_hz


def parse_xy(value: str) -> tuple[float, float]:
    left, right = value.split(",", 1)
    return float(left), float(right)


def parse_roi(value: str) -> tuple[int, int, int, int]:
    x, y, w, h = (int(part) for part in value.split(",", 3))
    return x, y, w, h


def build_mask(frame: np.ndarray, color: str, hsv_min: str | None, hsv_max: str | None) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if hsv_min and hsv_max:
        lo = np.array([int(v) for v in hsv_min.split(",")], dtype=np.uint8)
        hi = np.array([int(v) for v in hsv_max.split(",")], dtype=np.uint8)
        mask = cv2.inRange(hsv, lo, hi)
    elif color == "red":
        lo1, hi1, lo2, hi2 = HSV_PRESETS["red"]
        mask = cv2.inRange(hsv, np.array(lo1), np.array(hi1))
        mask |= cv2.inRange(hsv, np.array(lo2), np.array(hi2))
    else:
        lo, hi = HSV_PRESETS[color]
        mask = cv2.inRange(hsv, np.array(lo), np.array(hi))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_bob_center(mask: np.ndarray, min_area: float) -> tuple[float, float] | None:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < min_area:
        return None
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None
    return moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]


def iter_video_samples(args: argparse.Namespace) -> Iterable[tuple[float, float, float]]:
    source: str | int = args.video if args.video else args.camera
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video source: {source}")

    fps = args.fps or cap.get(cv2.CAP_PROP_FPS) or 30.0
    frame_index = 0
    roi = args.roi

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = frame_index / fps
        frame_index += 1
        if args.duration and t > args.duration:
            break

        x0 = y0 = 0
        work = frame
        if roi:
            x0, y0, w, h = roi
            work = frame[y0 : y0 + h, x0 : x0 + w]

        mask = build_mask(work, args.color, args.hsv_min, args.hsv_max)
        center = find_bob_center(mask, args.min_area)
        if center is None:
            continue
        yield t, center[0] + x0, center[1] + y0

    cap.release()


def iter_image_samples(args: argparse.Namespace) -> Iterable[tuple[float, float, float]]:
    extensions = {".bmp", ".jpg", ".jpeg", ".png"}
    paths = sorted(path for path in args.images.iterdir() if path.suffix.lower() in extensions)
    if not paths:
        raise RuntimeError(f"No image files found in: {args.images}")
    if not args.fps:
        raise RuntimeError("--fps is required when analyzing an image sequence.")

    roi = args.roi
    for index, path in enumerate(paths):
        t = index / args.fps
        if args.duration and t > args.duration:
            break
        frame = cv2.imread(str(path))
        if frame is None:
            continue

        x0 = y0 = 0
        work = frame
        if roi:
            x0, y0, w, h = roi
            work = frame[y0 : y0 + h, x0 : x0 + w]

        mask = build_mask(work, args.color, args.hsv_min, args.hsv_max)
        center = find_bob_center(mask, args.min_area)
        if center is None:
            continue
        yield t, center[0] + x0, center[1] + y0


def fit_motion(t: np.ndarray, x: np.ndarray) -> FitResult:
    if t.size < 10:
        raise ValueError("Need at least 10 tracked points to fit motion.")

    t0 = t - t[0]
    centered = x - np.mean(x)
    dt = float(np.median(np.diff(t0)))
    freqs = np.fft.rfftfreq(centered.size, dt)
    spectrum = np.abs(np.fft.rfft(centered))
    valid = freqs > 0
    guess = float(freqs[valid][np.argmax(spectrum[valid])])

    lo = max(0.02, guess * 0.5)
    hi = max(lo * 1.1, guess * 1.5)
    best: tuple[float, np.ndarray, float] | None = None

    for freq in np.linspace(lo, hi, 500):
        omega_t = 2.0 * math.pi * freq * t0
        design = np.column_stack([np.ones_like(t0), np.cos(omega_t), np.sin(omega_t)])
        coeff, *_ = np.linalg.lstsq(design, x, rcond=None)
        residual = x - design @ coeff
        rmse = float(np.sqrt(np.mean(residual * residual)))
        if best is None or rmse < best[2]:
            best = (float(freq), coeff, rmse)

    assert best is not None
    freq, coeff, rmse = best
    offset, cos_c, sin_c = (float(v) for v in coeff)
    amplitude = math.hypot(cos_c, sin_c)
    phase = math.atan2(-sin_c, cos_c)
    return FitResult(offset, amplitude, freq, phase, rmse)


def write_csv(path: Path, rows: list[tuple[float, float, float]]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "x_px", "y_px"])
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--video", help="Video file recorded from the K10 camera.")
    source.add_argument("--camera", type=int, help="Camera index if K10 appears as a computer camera.")
    source.add_argument("--images", type=Path, help="Directory of K10 TF-card photos named in capture order.")
    parser.add_argument("--fps", type=float, help="Override video FPS; required for --images.")
    parser.add_argument("--duration", type=float, help="Analyze only the first N seconds.")
    parser.add_argument("--color", choices=HSV_PRESETS.keys(), default="bright")
    parser.add_argument("--hsv-min", help="Manual HSV lower bound, e.g. 0,80,50.")
    parser.add_argument("--hsv-max", help="Manual HSV upper bound, e.g. 20,255,255.")
    parser.add_argument("--roi", type=parse_roi, help="x,y,w,h crop rectangle.")
    parser.add_argument("--min-area", type=float, default=40.0)
    parser.add_argument("--pivot", type=parse_xy, help="Pivot pixel coordinate x,y.")
    parser.add_argument("--length-m", type=float, help="Pendulum length from pivot to bob center, meters.")
    parser.add_argument("--csv", type=Path, help="Write tracked points to CSV.")
    args = parser.parse_args()

    rows = list(iter_image_samples(args) if args.images else iter_video_samples(args))
    if not rows:
        raise RuntimeError("No pendulum bob positions were tracked. Try --color, --hsv-min/--hsv-max, --roi, or --min-area.")
    if args.csv:
        write_csv(args.csv, rows)

    data = np.array(rows, dtype=float)
    t = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    fit = fit_motion(t, x)

    print("单摆拟合模型:")
    print(f"  x(t) = {fit.offset_px:.3f} + {fit.amplitude_px:.3f} cos(2π*{fit.frequency_hz:.5f}*t + {fit.phase_rad:.3f}) px")
    print(f"  角频率 omega = {fit.omega:.5f} rad/s")
    print(f"  频率 f = {fit.frequency_hz:.5f} Hz")
    print(f"  周期 T = {fit.period_s:.5f} s")
    print(f"  振幅 A = {fit.amplitude_px:.3f} px")
    print(f"  拟合 RMSE = {fit.rmse_px:.3f} px")
    print(f"  有效跟踪点 = {len(rows)}")

    if args.pivot:
        pivot_x, pivot_y = args.pivot
        length_px = float(np.median(np.hypot(x - pivot_x, y - pivot_y)))
        theta_amp = math.asin(min(0.999999, fit.amplitude_px / length_px))
        print(f"  摆长 L = {length_px:.3f} px")
        print(f"  角振幅 theta0 = {theta_amp:.5f} rad = {math.degrees(theta_amp):.3f} deg")
        if args.length_m:
            print(f"  线振幅 s0 = {args.length_m * theta_amp:.5f} m")

    if args.length_m:
        g = fit.omega * fit.omega * args.length_m
        print(f"  g = {g:.5f} m/s^2  (小角度公式 g=(2πf)^2 L)")


if __name__ == "__main__":
    main()
