"""Offline forensic analysis of Flight_Data Test2_3010; never edits source logs.

Run from repo root with Python + numpy/pandas/scipy/matplotlib.
SciPy/Matplotlib are missing from the bundled runtime on this host; the installed
scientific Python runtime supplies those capabilities. Outputs are derived
analysis only, not flight configuration or a modification of firmware behavior.
"""
from __future__ import annotations

import csv
import hashlib
import json
import platform
import re
import shutil
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy
from scipy.optimize import least_squares
from scipy.signal import find_peaks, lombscargle

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "Flight_Data"
OUT = DATA / "Analysis_Test2_3010"
OUT.mkdir(parents=True, exist_ok=True)
FIG = OUT / "figures"
FIG.mkdir(exist_ok=True)
G0 = 9.80665
MAG_B = np.array([-5.84, 3.08, 11.92])
MAG_M = np.array([[1.001, .012, .009], [.012, 1.006, -.001], [.009, -.001, .994]])
RNG = np.random.default_rng(20261101)
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .23,
                     "figure.dpi": 120, "savefig.dpi": 160})


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def jsave(name, value):
    def convert(v):
        if isinstance(v, np.ndarray): return v.tolist()
        if isinstance(v, np.generic): return v.item()
        if isinstance(v, Path): return str(v)
        raise TypeError(type(v))
    (OUT / name).write_text(json.dumps(value, indent=2, default=convert, allow_nan=False), encoding="utf-8")


def checksum(message):
    result = 0
    for char in message: result ^= ord(char)
    return f"{result:02X}"


def read_gnss():
    records, audit = [], []
    with (DATA / "gnss.csv").open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        for line, row in enumerate(reader, 2):
            reasons = []
            if len(row) != len(header):
                audit.append({"source_line": line, "accepted": False, "reasons": "width"})
                continue
            try:
                hh, mm, ss = int(row[2][:2]), int(row[2][2:4]), float(row[2][4:])
                assert 0 <= hh < 24 and 0 <= mm < 60 and 0 <= ss < 60
                utc = hh * 3600 + mm * 60 + ss
            except (ValueError, AssertionError):
                reasons.append("utc")
            if not re.fullmatch(r"\d{6}\.\d+", row[2]):
                reasons.append("utc_format")
            if not re.fullmatch(r"\d{4}\.\d+", row[3]) or not re.fullmatch(r"\d{5}\.\d+", row[5]):
                reasons.append("coordinate_format")
            if row[4] not in ["N", "S"] or row[6] not in ["E", "W"]:
                reasons.append("hemisphere")
            if (row[11] != "M" or row[13] != "M" or row[17] != "T" or
                row[19] != "M" or row[21] != "N" or row[23] != "K" or
                row[24] not in ["A", "D", "E", "N", "S", "F", "R", "P"]):
                reasons.append("units_or_mode")
            try:
                values = {header[i]: float(row[i]) for i in [1, 3, 5, 7, 8, 9, 10, 12, 16, 20, 22]}
                assert all(np.isfinite(list(values.values())))
                assert 0 <= values["true_course_deg"] <= 360
                assert 0 <= values["speed_knots"] < 100
                assert 0 <= values["hdop"] < 100
                assert abs(values["speed_knots"] * 1.852 - values["speed_kmh"]) < .04
                for idx, max_degree in [(3, 90), (5, 180)]:
                    degree = int(float(row[idx]) / 100)
                    minute = float(row[idx]) - degree * 100
                    assert 0 <= degree <= max_degree and 0 <= minute < 60
            except (ValueError, AssertionError):
                reasons.append("numeric_or_range")
            # Full exact fields reconstructed; no repair or replacement of values.
            if checksum("GNGGA," + ",".join(row[2:16])) != row[25]:
                reasons.append("gga_checksum")
            if checksum("GNVTG," + ",".join(row[16:25])) != row[26]:
                reasons.append("vtg_checksum")
            audit.append({"source_line": line, "accepted": not reasons, "reasons": ";".join(reasons)})
            if not reasons:
                record = dict(zip(header, row))
                record.update(values)
                record.update(source_line=line, utc_s=utc, t_s=values["boot_ms"] / 1000)
                records.append(record)
    g = pd.DataFrame(records)
    g["utc_minus_boot_s"] = g.utc_s - g.t_s
    # A UTC-to-boot offset discontinuity catches the first reset that decreasing
    # boot_ms alone misses: old session ends before the new session's first fix.
    g["session"] = (g.utc_minus_boot_s.diff().abs() > 2).cumsum().astype(int)
    g["lat_deg"] = np.floor(g.latitude_nmea / 100) + (g.latitude_nmea % 100) / 60
    g["lon_deg"] = np.floor(g.longitude_nmea / 100) + (g.longitude_nmea % 100) / 60
    g.loc[g.latitude_hemisphere == "S", "lat_deg"] *= -1
    g.loc[g.longitude_hemisphere == "W", "lon_deg"] *= -1
    g["speed_mps"] = g.speed_kmh / 3.6
    g["vN_mps"] = g.speed_mps * np.cos(np.deg2rad(g.true_course_deg))
    g["vE_mps"] = g.speed_mps * np.sin(np.deg2rad(g.true_course_deg))
    audit = pd.DataFrame(audit)
    audit.to_csv(OUT / "gnss_row_audit.csv", index=False)
    g.to_csv(OUT / "gnss_validated.csv", index=False)
    return g, audit


def interval_stats(t):
    dt = np.diff(np.asarray(t))
    return {"n": len(t), "duration_s": float(t.iloc[-1] - t.iloc[0]),
            "median_dt_s": float(np.median(dt)), "p95_dt_s": float(np.quantile(dt, .95)),
            "p99_dt_s": float(np.quantile(dt, .99)), "max_dt_s": float(dt.max()),
            "effective_rate_hz": float((len(t) - 1) / (t.iloc[-1] - t.iloc[0])),
            "gaps_over_0_2_s": int((dt > .2 + 1e-9).sum()),
            "duplicate_or_backward_intervals": int((dt <= 0).sum())}


def circle_fit(xy):
    centre0 = xy.mean(axis=0)
    z = xy - centre0
    design = np.column_stack([2 * z, np.ones(len(z))])
    beta = np.linalg.lstsq(design, np.sum(z * z, axis=1), rcond=None)[0]
    linear_centre = centre0 + beta[:2]
    linear_radius = np.sqrt(max(0, beta[2] + np.sum(beta[:2] ** 2)))
    fit = least_squares(lambda p: np.linalg.norm(xy - p[:2], axis=1) - p[2],
                        np.r_[linear_centre, linear_radius],
                        bounds=([-np.inf, -np.inf, 0], [np.inf] * 3), max_nfev=1000)
    angles = np.sort(np.mod(np.arctan2(xy[:, 1] - fit.x[1], xy[:, 0] - fit.x[0]), 2 * np.pi))
    coverage = 2 * np.pi - np.max(np.diff(np.r_[angles, angles[0] + 2 * np.pi]))
    return {"WN_mps": float(fit.x[0]), "WE_mps": float(fit.x[1]), "turn_airspeed_mps": float(fit.x[2]),
            "wind_speed_mps": float(np.linalg.norm(fit.x[:2])),
            "wind_towards_deg": float(np.rad2deg(np.arctan2(fit.x[1], fit.x[0])) % 360),
            "radial_rmse_mps": float(np.sqrt(np.mean(fit.fun ** 2))),
            "radial_rmse_fraction": float(np.sqrt(np.mean(fit.fun ** 2)) / fit.x[2]),
            "fitted_direction_coverage_deg": float(np.rad2deg(coverage)),
            "centred_design_condition": float(np.linalg.cond(design)),
            "linear_WN_mps": float(linear_centre[0]), "linear_WE_mps": float(linear_centre[1]),
            "linear_radius_mps": float(linear_radius), "converged": bool(fit.success)}


def sphere_fit(xyz):
    mean = xyz.mean(axis=0)
    z = xyz - mean
    design = np.column_stack([2 * z, np.ones(len(z))])
    beta = np.linalg.lstsq(design, np.sum(z * z, axis=1), rcond=None)[0]
    init = np.r_[mean + beta[:3], np.sqrt(max(0, beta[3] + np.sum(beta[:3] ** 2)))]
    fit = least_squares(lambda p: np.linalg.norm(xyz - p[:3], axis=1) - p[3], init,
                        bounds=([-np.inf] * 3 + [0], [np.inf] * 4), max_nfev=2000)
    return {"offset_uT": fit.x[:3].tolist(), "field_radius_uT": float(fit.x[3]),
            "radial_rmse_uT": float(np.sqrt(np.mean(fit.fun ** 2))),
            "condition": float(np.linalg.cond(design))}


def phase(df, start, end):
    return df[(df.t_s >= start) & (df.t_s <= end)].copy()


def line_fit(t, y):
    slope, intercept = np.polyfit(t, y, 1)
    rms = np.sqrt(np.mean((y - slope * t - intercept) ** 2))
    return {"slope_mps": float(slope), "intercept": float(intercept), "residual_rmse_m": float(rms), "n": len(t)}


def savefig(name):
    plt.savefig(FIG / name, bbox_inches="tight")
    plt.close()


def main():
    original_hashes = {name: sha(DATA / name) for name in ["gnss.csv", "sensors.csv", "restarts.csv"]}
    s = pd.read_csv(DATA / "sensors.csv")
    assert s.notna().all().all(), "Sensor NaNs require explicit handling"
    s["source_line"] = np.arange(len(s)) + 2
    s["session"] = (s.boot_ms.diff() < 0).cumsum().astype(int)
    s["t_s"] = s.boot_ms / 1000
    g, audit = read_gnss()
    assert s.session.nunique() == g.session.nunique() == 6
    sessions = []
    for session, ss in s.groupby("session"):
        gg = g[g.session == session]
        sessions.append({"session": int(session), "sensor_rows": len(ss), "valid_gnss_rows": len(gg),
                         "sensor_start_s": ss.t_s.iloc[0], "sensor_end_s": ss.t_s.iloc[-1],
                         "first_valid_gnss_utc": gg.utc_time.iloc[0], "last_valid_gnss_utc": gg.utc_time.iloc[-1],
                         "utc_minus_boot_median_s": gg.utc_minus_boot_s.median(),
                         "baro_min_m": ss.relative_alt_m.min(), "baro_max_m": ss.relative_alt_m.max()})
    pd.DataFrame(sessions).to_csv(OUT / "boot_sessions.csv", index=False)
    f = s[s.session == 5].copy()
    gg = g[g.session == 5].copy()
    assert (np.diff(f.t_s) > 0).all() and (np.diff(gg.t_s) > 0).all()
    assert (gg.fix_quality > 0).all()
    accel_cols = ["ax_ms2", "ay_ms2", "az_ms2"]
    gyro_cols = ["gx_dps", "gy_dps", "gz_dps"]
    mag_cols = ["mx_uT", "my_uT", "mz_uT"]
    for df in [s, f]:
        df["accel_norm_g"] = np.linalg.norm(df[accel_cols], axis=1) / G0
        df["gyro_norm_dps"] = np.linalg.norm(df[gyro_cols], axis=1)
    # Two-second quiet blocks on pressure-stable ground. These select low-motion
    # candidate calibration windows, not a certified knowledge of true attitude.
    windows = []
    static_rows = []
    for sid, block in s.groupby("session"):
        limit = 64 if sid == 5 else block.t_s.max()
        candidate = block[(block.t_s >= 8) & (block.t_s <= limit)].copy()
        candidate["block"] = (candidate.t_s // 2).astype(int)
        for _, w in candidate.groupby("block"):
            if len(w) < 12 or w.t_s.max() - w.t_s.min() < 1.4: continue
            if (w[gyro_cols].std().max() < .18 and w[accel_cols].std().max() < .035
                and w.gyro_norm_dps.mean() < .6 and w.relative_alt_m.std() < .15):
                windows.append({"session": int(sid), "start_s": w.t_s.min(), "end_s": w.t_s.max(),
                                "n": len(w), **{c: w[c].mean() for c in accel_cols + gyro_cols},
                                "gyro_std_max_dps": w[gyro_cols].std().max(),
                                "accel_std_max_ms2": w[accel_cols].std().max()})
                static_rows.extend(w.index.tolist())
    static = s.loc[static_rows].copy()
    pd.DataFrame(windows).to_csv(OUT / "stationary_windows.csv", index=False)
    gyro_cal = []
    for sid, block in static.groupby("session"):
        bs = np.array([list(w[c] for c in gyro_cols) for w in windows if w["session"] == sid])
        means = (np.array([bs[RNG.integers(0, len(bs), len(bs))].mean(axis=0) for _ in range(1000)])
                 if len(bs) >= 2 else None)
        gyro_cal.append({"session": int(sid), "n": len(block), "blocks_2s": len(bs),
                         "bias_dps": block[gyro_cols].mean().to_numpy().tolist(),
                         "sample_std_dps": block[gyro_cols].std().to_numpy().tolist(),
                         "block_bootstrap_95_lo_dps": np.quantile(means, .025, axis=0).tolist() if means is not None else None,
                         "block_bootstrap_95_hi_dps": np.quantile(means, .975, axis=0).tolist() if means is not None else None,
                         "mean_accel_ms2": block[accel_cols].mean().to_numpy().tolist(),
                         "median_accel_norm_ms2": float(np.median(block.accel_norm_g) * G0)})
    fc = next(c for c in gyro_cal if c["session"] == 5)
    bias = np.array(fc["bias_dps"])
    f[["gx_bias_corrected_dps", "gy_bias_corrected_dps", "gz_bias_corrected_dps"]] = f[gyro_cols] - bias
    ground = static[static.session == 5]
    pressure_ground = float(ground.pressure_pa.median())
    altitude_ground = float(ground.relative_alt_m.median())
    logged_p0 = f.pressure_pa / (1 - f.relative_alt_m / 44330) ** (1 / .1903)
    f["pressure_ground_relative_m"] = 44330 * (1 - (f.pressure_pa / pressure_ground) ** .1903)
    f["accel_band_admitted"] = f.accel_norm_g.between(.8, 1.2)
    f["mag_norm_raw_uT"] = np.linalg.norm(f[mag_cols], axis=1)
    f["mag_norm_other_pcb_cal_uT"] = np.linalg.norm((f[mag_cols].to_numpy() - MAG_B) @ MAG_M.T, axis=1)
    descent_mag = phase(f, 168, 181.65)
    mag_fit = sphere_fit(descent_mag[mag_cols].to_numpy())
    f["mag_norm_diagnostic_fit_uT"] = np.linalg.norm(f[mag_cols] - np.array(mag_fit["offset_uT"]), axis=1)
    mag_validation = {}
    for label, a, b in [("early_descent", 168, 175), ("late_descent", 175, 181.65)]:
        train = phase(f, a, b)
        model = sphere_fit(train[mag_cols].to_numpy())
        test = phase(f, 175 if label == "early_descent" else 168,
                     181.65 if label == "early_descent" else 175)
        residual = np.linalg.norm(test[mag_cols] - np.array(model["offset_uT"]), axis=1) - model["field_radius_uT"]
        mag_validation[label] = {**model, "other_half_rmse_uT": float(np.sqrt(np.mean(residual ** 2)))}
    calibration = {"scope": "Offline candidates for this raw log only; not automatically flight configuration",
                   "gyro": {"per_session": gyro_cal, "flight_session": 5,
                            "correction": "gyro_corrected_dps = raw_dps - bias_dps; multiply by pi/180 for rad/s",
                            "scale_cross_axis_temperature_identifiable": False},
                   "accelerometer": {"full_affine_calibration_identifiable": False,
                                     "known_pose_provided_for_this_mount": False,
                                     "flight_stationary_mean_ms2": fc["mean_accel_ms2"],
                                     "stationary_norm_ms2": fc["median_accel_norm_ms2"],
                                     "illustrative_uniform_gain_if_zero_bias": G0 / fc["median_accel_norm_ms2"],
                                     "uniform_gain_applied": False,
                                     "bias_applied": False},
                   "barometer": {"ground_reference_pressure_pa": pressure_ground,
                                  "ground_logged_relative_altitude_median_m": altitude_ground,
                                  "inferred_logger_reference_pressure_pa": float(logged_p0.median()),
                                  "reference_pressure_scatter_pa": float(logged_p0.std()),
                                  "pressure_absolute_accuracy_calibrated": False},
                   "mag_reference_other_pcb": {"offset_uT": MAG_B, "mapping": MAG_M, "field_uT": 25.25,
                                               "representative_of_flight_pcb": False},
                   "mag_diagnostic_hard_iron_only": {**mag_fit, "cross_validation": mag_validation,
                                                     "flight_approved": False, "absolute_heading_validated": False}}
    jsave("calibration_candidates.json", calibration)

    lat0 = np.deg2rad(phase(gg, 8, 60).lat_deg.median())
    lon0 = phase(gg, 8, 60).lon_deg.median()
    a, e2 = 6378137.0, .00669437999014
    rn = a * (1 - e2) / (1 - e2 * np.sin(lat0) ** 2) ** 1.5
    re = a / np.sqrt(1 - e2 * np.sin(lat0) ** 2)
    gg["north_m"] = (np.deg2rad(gg.lat_deg) - lat0) * rn
    gg["east_m"] = np.deg2rad(gg.lon_deg - lon0) * re * np.cos(lat0)
    gg["baro_height_interpolated_m"] = np.interp(gg.t_s, f.t_s, f.relative_alt_m)
    gnss_ground = float(phase(gg, 8, 60).altitude.median())
    gg["gnss_ground_relative_m"] = gg.altitude - gnss_ground
    circle_windows = []
    for a, b in [(165, 183.1), (166, 183.1), (168, 183.1), (170, 181.5), (172, 181.5),
                 (169, 176), (171, 176), (173, 176), (176, 181.5), (170, 174), (174, 178), (178, 181.5)]:
        w = phase(gg, a, b)
        fit = circle_fit(w[["vN_mps", "vE_mps"]].to_numpy())
        vv = w[["vN_mps", "vE_mps"]].to_numpy() - [fit["WN_mps"], fit["WE_mps"]]
        angles = np.unwrap(np.arctan2(vv[:, 1], vv[:, 0]))
        fit.update(start_s=a, end_s=b, n=len(w),
                   apparent_air_direction_rate_hz=float(np.polyfit(w.t_s, angles, 1)[0] / (2 * np.pi)))
        circle_windows.append(fit)
    fits = pd.DataFrame(circle_windows)
    fits.to_csv(OUT / "circle_fit_windows.csv", index=False)
    primary = next(z for z in circle_windows if z["start_s"] == 170 and z["end_s"] == 181.5)
    w = phase(gg, 170, 181.5).copy()
    w["block"] = (w.t_s - 170).astype(int)
    blocks = [z[["vN_mps", "vE_mps"]].to_numpy() for _, z in w.groupby("block")]
    boot_params = []
    for _ in range(200):
        sample = np.concatenate([blocks[i] for i in RNG.integers(0, len(blocks), len(blocks))])
        fit = circle_fit(sample)
        if fit["converged"]:
            boot_params.append([fit[k] for k in ["WN_mps", "WE_mps", "turn_airspeed_mps"]])
    primary["conditional_block_bootstrap_95_lo"] = np.quantile(boot_params, .025, axis=0).tolist()
    primary["conditional_block_bootstrap_95_hi"] = np.quantile(boot_params, .975, axis=0).tolist()
    primary["bootstrap_replicates_converged"] = len(boot_params)
    # Cross-window consistency is more informative than same-window residual.
    cross_errors = []
    for train_a, train_b, test_a, test_b in [(171, 176, 178, 181.5), (178, 181.5, 171, 176)]:
        model = next(z for z in circle_windows if z["start_s"] == train_a and z["end_s"] == train_b)
        test = phase(gg, test_a, test_b)
        residual = np.linalg.norm(test[["vN_mps", "vE_mps"]] - [model["WN_mps"], model["WE_mps"]], axis=1) - model["turn_airspeed_mps"]
        cross_errors.append({"train": [train_a, train_b], "test": [test_a, test_b],
                             "radial_rmse_mps": float(np.sqrt(np.mean(residual ** 2)))})
    # Derivative cross-checks and constant-velocity prediction residuals. GGA is
    # another output of the SAME receiver, so these are not independent truth.
    prediction_rows, prediction_summary = [], []
    for horizon in [.1, .5, 1.0, 2.0]:
        for row in phase(gg, 168, 181.5 - horizon).itertuples():
            future = np.array([np.interp(row.t_s + horizon, gg.t_s, gg.north_m),
                               np.interp(row.t_s + horizon, gg.t_s, gg.east_m)])
            predicted = np.array([row.north_m, row.east_m]) + horizon * np.array([row.vN_mps, row.vE_mps])
            error = future - predicted
            prediction_rows.append({"horizon_s": horizon, "start_s": row.t_s,
                                    "error_N_m": error[0], "error_E_m": error[1],
                                    "horizontal_error_m": np.linalg.norm(error)})
        subset = pd.DataFrame([r for r in prediction_rows if r["horizon_s"] == horizon])
        prediction_summary.append({"horizon_s": horizon, "n": len(subset),
                                   "median_error_m": float(subset.horizontal_error_m.median()),
                                   "p95_error_m": float(subset.horizontal_error_m.quantile(.95)),
                                   "max_error_m": float(subset.horizontal_error_m.max())})
    pd.DataFrame(prediction_rows).to_csv(OUT / "propagation_residuals.csv", index=False)
    # Descriptive GNSS/baro lag+gain fit, explicitly NOT a receiver-latency calibration.
    lag_fits = []
    for lag in np.arange(-10, 70.01, .1):
        subset = phase(gg, 70, 181)
        subset = subset[(subset.t_s - lag >= f.t_s.min()) & (subset.t_s - lag <= f.t_s.max())]
        x = np.interp(subset.t_s - lag, f.t_s, f.relative_alt_m)
        design = np.column_stack([np.ones(len(x)), x])
        offset, scale = np.linalg.lstsq(design, subset.altitude, rcond=None)[0]
        rmse = np.sqrt(np.mean((design @ [offset, scale] - subset.altitude) ** 2))
        lag_fits.append({"apparent_lag_s": float(lag), "offset_m": float(offset),
                         "scale": float(scale), "rmse_m": float(rmse)})
    best_lag = min(lag_fits, key=lambda x: x["rmse_m"])
    no_lag = min(lag_fits, key=lambda x: abs(x["apparent_lag_s"]))
    pd.DataFrame(lag_fits).to_csv(OUT / "height_lag_sensitivity.csv", index=False)
    phases = [("ground", 8, 64), ("first_climb", 70, 110), ("intermediate_hover", 112, 130),
              ("second_climb", 136, 150), ("top_hover", 154, 164),
              ("release_inflation", 166.384, 167.972), ("damaged_descent_dense", 168, 181.65),
              ("descent_including_sparse_tail", 168, 187.9)]
    phase_stats = []
    for name, a, b in phases:
        df = phase(f, a, b)
        phase_stats.append({"phase": name, "start_s": a, "end_s": b,
                            **line_fit(df.t_s.to_numpy(), df.relative_alt_m.to_numpy()),
                            "accel_band_admission_fraction": float(df.accel_band_admitted.mean()),
                            "accel_g_median": float(df.accel_norm_g.median()),
                            "accel_g_min": float(df.accel_norm_g.min()), "accel_g_max": float(df.accel_norm_g.max()),
                            "gyro_axis_abs_max_dps": df[gyro_cols].abs().max().to_numpy().tolist()})
    pd.DataFrame(phase_stats).to_csv(OUT / "phase_statistics.csv", index=False)
    # Spectra are of RAW SENSOR CHANNELS, not attitude or canopy motion truth.
    freq = np.linspace(.15, 4.5, 2600)
    spectral_rows = []
    spectrum_curves = {}
    for a, b in [(168, 181.65), (168, 174), (174, 181.65)]:
        df = phase(f, a, b)
        t = df.t_s.to_numpy()
        for column in gyro_cols + mag_cols + accel_cols:
            signal = df[column].to_numpy()
            signal = signal - np.polyval(np.polyfit(t - t[0], signal, 1), t - t[0])
            power = lombscargle(t - t[0], signal, 2 * np.pi * freq, normalize=True)
            peaks, _ = find_peaks(power, distance=35)
            ranked = sorted(peaks, key=lambda k: power[k], reverse=True)[:3]
            for rank, j in enumerate(ranked, 1):
                spectral_rows.append({"start_s": a, "end_s": b, "channel": column,
                                      "rank": rank, "peak_hz": freq[j], "normalized_power": power[j],
                                      "rayleigh_resolution_hz": 1 / (t[-1] - t[0])})
            if a == 168 and b == 181.65:
                spectrum_curves[column] = power
    pd.DataFrame(spectral_rows).to_csv(OUT / "spectral_peaks.csv", index=False)
    f.to_csv(OUT / "flight_sensors_derived.csv", index=False)
    gg.to_csv(OUT / "flight_gnss_derived.csv", index=False)
    # Explicitly flag long record intervals; they are not sensor-validity flags.
    gaps = f.loc[f.t_s.diff() > .2 + 1e-9, ["source_line", "t_s", "relative_alt_m"]].copy()
    gaps["gap_s"] = f.t_s.diff().loc[gaps.index]
    gaps.to_csv(OUT / "sensor_record_gaps.csv", index=False)
    baseline_deltas = []
    for sid, df in s.groupby("session"):
        delta = df[mag_cols].to_numpy() - df[["dmx_uT", "dmy_uT", "dmz_uT"]].to_numpy()
        baseline_deltas.append({"session": int(sid), "mean_reference_uT": delta.mean(axis=0).tolist(),
                                "max_reference_spread_uT": float(np.ptp(delta, axis=0).max())})
    descent_gnss = phase(gg, 166.384, 181.65)
    metrics = {"inputs_sha256": original_hashes,
               "runtime": {"python": platform.python_version(), "numpy": np.__version__,
                           "pandas": pd.__version__, "scipy": scipy.__version__, "matplotlib": matplotlib.__version__},
               "sensor_rows": len(s), "gnss_rows": len(audit), "gnss_accepted": len(g),
               "gnss_rejected": int((~audit.accepted).sum()),
               "gnss_failure_counts_nonexclusive": dict(Counter(x for r in audit.reasons for x in r.split(";") if x)),
               "all_sensor_status_flags_one": bool((s[["imu_ok", "mag_ok", "bmp_ok"]] == 1).all().all()),
               "sessions": sessions, "flight_sensor_timing": interval_stats(f.t_s),
               "flight_gnss_timing": interval_stats(gg.t_s),
               "flight_gnss_hdop_range": [float(gg.hdop.min()), float(gg.hdop.max())],
               "flight_gnss_satellites_range": [int(gg.satellites_used.min()), int(gg.satellites_used.max())],
               "descent_low_speed_example": {"window_s": [166.384, 181.65],
                                              "n": len(descent_gnss),
                                              "below_0_5_mps": int((descent_gnss.speed_mps < .5).sum()),
                                              "min_mps": float(descent_gnss.speed_mps.min())},
               "flight_gnss_baseline_msl_m": gnss_ground,
               "baro_peak": f.loc[f.relative_alt_m.idxmax(), ["source_line", "t_s", "relative_alt_m"]].to_dict(),
               "gnss_peak": gg.loc[gg.altitude.idxmax(), ["source_line", "t_s", "altitude", "gnss_ground_relative_m"]].to_dict(),
               "peak_accel": f.loc[f.accel_norm_g.idxmax(), ["source_line", "t_s", "accel_norm_g"]].to_dict(),
               "minimum_release_accel": phase(f, 166, 168).loc[phase(f, 166, 168).accel_norm_g.idxmin(), ["t_s", "accel_norm_g"]].to_dict(),
               "last_sensor": f.iloc[-1][["source_line", "t_s", "relative_alt_m", "gyro_norm_dps"]].to_dict(),
               "last_gnss": gg.iloc[-1][["source_line", "t_s", "altitude", "speed_mps"]].to_dict(),
               "phases": phase_stats, "primary_circle_fit": primary, "circle_cross_window_errors": cross_errors,
               "propagation_against_same_receiver": prediction_summary,
               "descriptive_height_alignment": {"best_lag_plus_scale": best_lag, "zero_lag_plus_scale": no_lag},
               "dmag_reference_diagnostics": baseline_deltas,
               "mag_supplied_cal_norm_preflight_median_uT": float(phase(f, 8, 60).mag_norm_other_pcb_cal_uT.median()),
               "ground_pressure_std_pa": float(ground.pressure_pa.std()),
               "ground_baro_std_m": float(ground.relative_alt_m.std())}
    jsave("metrics.json", metrics)

    # Break lines across long recording gaps; retain isolated sample markers.
    def trace(ax, frame, x, y, **kwargs):
        xx = frame[x].to_numpy(); yy = frame[y].to_numpy(dtype=float).copy()
        bad = np.r_[False, np.diff(xx) > .25]
        continuous = yy.copy(); continuous[bad] = np.nan
        line, = ax.plot(xx, continuous, **kwargs)
        if bad.any(): ax.plot(xx[bad], yy[bad], "o", ms=3, color=line.get_color())
        return line

    # 1. Whole-flight overview with no interpolation hiding missing IMU tail.
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, layout="constrained")
    trace(axes[0], f, "t_s", "relative_alt_m", label="Pressure-derived relative height")
    trace(axes[0], gg, "t_s", "gnss_ground_relative_m", label="GNSS GGA minus ground median")
    axes[0].set(ylabel="Height (m)", title="Last boot session: ascent, direct deployment and damaged descent")
    axes[0].legend(loc="upper left")
    trace(axes[1], gg, "t_s", "speed_mps", color="tab:green", label="GNSS speed")
    axes[1].set(ylabel="Ground speed (m/s)")
    axes[1].legend(loc="upper left")
    trace(axes[2], f, "t_s", "gyro_norm_dps", color="tab:orange", label="Body angular-rate magnitude")
    axes[2].set(ylabel="Angular rate (deg/s)", xlabel="Seconds since last boot")
    for ax in axes:
        ax.axvline(166.384, color="black", ls="--", lw=1, label="Release-compatible transient")
        ax.axvspan(181.641, 187.821, color="grey", alpha=.12)
    axes[2].legend(loc="upper left")
    savefig("01_flight_overview.png")

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, layout="constrained")
    d = phase(f, 164, 171)
    axes[0].plot(d.t_s, d.accel_norm_g, "o-", ms=3)
    axes[0].axhspan(.8, 1.2, color="tab:green", alpha=.15)
    axes[0].set(ylabel="Specific-force norm (g)", title="Release/inflation-compatible transient; no logged HDRM event timestamp")
    for col, label in zip(gyro_cols, ["X", "Y", "Z"]): axes[1].plot(d.t_s, d[col], "o-", ms=2, label=label)
    axes[1].set(ylabel="Raw gyro (deg/s)")
    axes[1].legend(ncol=3)
    axes[2].plot(d.t_s, d.relative_alt_m, "o-", ms=3)
    axes[2].set(ylabel="Logged relative height (m)", xlabel="Seconds since last boot")
    for ax in axes:
        ax.axvline(166.384, color="black", ls="--", lw=1)
        ax.axvline(167.672, color="tab:red", ls=":", lw=1)
    savefig("02_release_transient.png")

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout="constrained")
    for ax, subset, title in [(axes[0], gg, "Whole last-session GNSS track"),
                               (axes[1], phase(gg, 166.384, 183.1), "Descent portion; last fix is not touchdown")]:
        sc = ax.scatter(subset.east_m, subset.north_m, c=subset.t_s, cmap="viridis", s=12)
        ax.plot(subset.east_m, subset.north_m, color="grey", alpha=.25)
        ax.set(xlabel="East from ground reference (m)", ylabel="North (m)", title=title)
        ax.set_aspect("equal", adjustable="datalim")
        fig.colorbar(sc, ax=ax, label="Seconds since boot", shrink=.8)
    savefig("03_gnss_trajectory.png")

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout="constrained")
    w = phase(gg, 168, 181.5)
    sc = axes[0].scatter(w.vE_mps, w.vN_mps, c=w.t_s, cmap="viridis", s=20)
    angle = np.linspace(0, 2*np.pi, 400)
    for a, b, color in [(170, 181.5, "black"), (171, 176, "tab:red"), (178, 181.5, "tab:blue")]:
        model = next(z for z in circle_windows if z["start_s"] == a and z["end_s"] == b)
        axes[0].plot(model["WE_mps"] + model["turn_airspeed_mps"] * np.sin(angle),
                     model["WN_mps"] + model["turn_airspeed_mps"] * np.cos(angle),
                     color=color, lw=1.5, label=f"Fit {a:g}-{b:g} s")
        axes[0].plot(model["WE_mps"], model["WN_mps"], "+", color=color, ms=10)
    axes[0].set(xlabel="GNSS East velocity (m/s)", ylabel="GNSS North velocity (m/s)",
                title="Velocity-space circles change with the fitting window")
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].legend(fontsize=8)
    fig.colorbar(sc, ax=axes[0], label="Seconds since boot", shrink=.8)
    apparent = np.hypot(w.vN_mps - primary["WN_mps"], w.vE_mps - primary["WE_mps"])
    axes[1].plot(w.t_s, apparent, "o-", ms=3, label="|Ground velocity - full-window fitted wind|")
    axes[1].axhline(primary["turn_airspeed_mps"], color="black", ls="--", label="Full-window fitted radius")
    axes[1].set(xlabel="Seconds since boot", ylabel="Apparent airspeed (m/s)",
                title="Apparent speed varies; healthy trim is not established")
    axes[1].legend(fontsize=8)
    savefig("04_velocity_circle_fits.png")

    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True, layout="constrained")
    for col, label in zip(gyro_cols, ["X", "Y", "Z"]): trace(axes[0], f, "t_s", col, label=label, lw=.9)
    axes[0].set(ylabel="Body gyro (deg/s)", title="Raw sensor dynamics and magnetometer calibration sensitivity")
    axes[0].legend(ncol=3)
    trace(axes[1], f, "t_s", "accel_norm_g", lw=.9)
    axes[1].axhspan(.8, 1.2, color="tab:green", alpha=.15, label="0.8-1.2 g admission band")
    axes[1].set(ylabel="Specific force (g)")
    axes[1].legend()
    trace(axes[2], f, "t_s", "mag_norm_raw_uT", label="Raw")
    trace(axes[2], f, "t_s", "mag_norm_other_pcb_cal_uT", label="Other-PCB calibration")
    trace(axes[2], f, "t_s", "mag_norm_diagnostic_fit_uT", label="Flight-only diagnostic sphere offset")
    axes[2].axhline(25.25, color="tab:red", ls="--", label="Other-PCB reference 25.25 uT")
    axes[2].set(xlabel="Seconds since boot", ylabel="Magnetic norm (uT)")
    axes[2].legend(fontsize=8, ncol=2)
    savefig("05_raw_sensor_dynamics.png")

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, layout="constrained")
    for index, label in enumerate(["X", "Y", "Z"]):
        ids = [z["session"] for z in gyro_cal]
        vals = [z["bias_dps"][index] for z in gyro_cal]
        lo = [max(0, z["bias_dps"][index] - z["block_bootstrap_95_lo_dps"][index])
              if z["block_bootstrap_95_lo_dps"] is not None else 0 for z in gyro_cal]
        hi = [max(0, z["block_bootstrap_95_hi_dps"][index] - z["bias_dps"][index])
              if z["block_bootstrap_95_hi_dps"] is not None else 0 for z in gyro_cal]
        axes[index].errorbar(ids, vals, yerr=[lo, hi], fmt="o", capsize=4)
        axes[index].axhline(0, color="grey", lw=.7)
        axes[index].set(ylabel=f"{label} mean (deg/s)")
    axes[0].set_title("Quiet-window gyro means; session 0 has only one block, no uncertainty estimate")
    axes[2].set(xlabel="Boot session (last-flight session = 5)", xticks=range(6))
    savefig("06_gyro_bias_candidates.png")

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True, layout="constrained")
    for column in gyro_cols: axes[0].plot(freq, spectrum_curves[column], label=column)
    for column in mag_cols: axes[1].plot(freq, spectrum_curves[column], label=column)
    axes[0].set(title="Detrended raw-channel spectra, 168-181.65 s (irregular-sample Lomb-Scargle)",
                ylabel="Normalized power")
    axes[1].set(xlabel="Frequency (Hz)", ylabel="Normalized power")
    for ax in axes: ax.legend(ncol=3); ax.set_xlim(.15, 4.5)
    savefig("07_sensor_spectra.png")

    # Reconciliation checks are independent of headline interpretation.
    synthetic_angles = np.linspace(0, 2 * np.pi, 200, endpoint=False)
    synthetic_velocity = np.array([2.0, -3.0]) + 4.0 * np.column_stack(
        [np.cos(synthetic_angles), np.sin(synthetic_angles)])
    recovered_circle = circle_fit(synthetic_velocity)
    assert np.allclose([recovered_circle[k] for k in ["WN_mps", "WE_mps", "turn_airspeed_mps"]],
                       [2.0, -3.0, 4.0], atol=1e-8)
    sphere_rng = np.random.default_rng(27)
    unit_vectors = sphere_rng.normal(size=(200, 3))
    unit_vectors /= np.linalg.norm(unit_vectors, axis=1)[:, None]
    recovered_sphere = sphere_fit(np.array([10.0, -20.0, 30.0]) + 45.0 * unit_vectors)
    assert np.allclose(recovered_sphere["offset_uT"], [10, -20, 30], atol=1e-8)
    assert abs(recovered_sphere["field_radius_uT"] - 45) < 1e-8
    assert len(audit) == len(g) + (~audit.accepted).sum()
    assert np.max(np.abs(gg.east_m)) < 1000 and np.max(np.abs(gg.north_m)) < 1000, "Inspect implausible local position"
    assert np.max(np.abs(gg.speed_knots * 1.852 - gg.speed_kmh)) < .04
    assert max(z["max_reference_spread_uT"] for z in baseline_deltas) < 1e-8
    assert original_hashes == {name: sha(DATA / name) for name in original_hashes}
    jsave("verification.json", {"originals_unchanged": True, "gnss_row_count_reconciled": True,
                                 "source_checksum_reconstruction": "GNGGA and GNVTG for accepted rows",
                                 "speed_units_cross_checked": True, "dmag_is_constant_reference_subtraction": True,
                                 "synthetic_circle_known_wind_speed_recovered": True,
                                 "synthetic_sphere_known_offset_field_recovered": True,
                                 "firmware_modified": False})
    print(json.dumps({"out": str(OUT), "gyro_calibration": fc, "metrics": metrics}, indent=2))


if __name__ == "__main__":
    main()
