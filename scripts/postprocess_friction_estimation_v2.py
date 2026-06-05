#!/usr/bin/env python3

"""Post-process friction estimation CSV into controller-ready parameters.

Pipeline implemented to match requested paper-style workflow:
1) Load captured CSV data.
2) Filter measured torque with 4th-order zero-phase Butterworth LPF.
3) Build rigid-body/controller prediction tau_hat from logged signals.
4) Compute friction residual delta_tau = tau_filt - tau_hat.
5) Fit sigmoidal friction model with Nelder-Mead (MATLAB fminsearch style).
6) Export /tmp outputs for controller use.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np
from scipy.optimize import minimize
from scipy.signal import butter, filtfilt

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover - optional dependency for plotting only.
    plt = None


@dataclass
class JointFitResult:
    joint: int
    count: int
    phi1: float
    phi2: float
    phi3: float
    rmse: float
    fvp: float
    fcp: float
    fvn: float
    fcn: float


@dataclass
class JointSignals:
    t: np.ndarray
    dq: np.ndarray
    tau_raw: np.ndarray
    tau_filt: np.ndarray
    tau_hat: np.ndarray
    residual: np.ndarray
    tau_compensated: np.ndarray
    tau_gravity: np.ndarray
    tau_inertia: np.ndarray
    tau_coriolis: np.ndarray
    tau_friction: np.ndarray
    tau_friction_model1: np.ndarray
    tau_friction_model2: np.ndarray
    tau_compensated_model1: np.ndarray
    tau_compensated_model2: np.ndarray


def parse_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def parse_vector(text: str, expected_len: int | None = None) -> List[float]:
    if text.strip() == "":
        return []
    vals = [float(x.strip()) for x in text.split(",") if x.strip() != ""]
    if expected_len is not None and len(vals) not in (0, expected_len):
        raise ValueError(
            f"Expected either 0 or {expected_len} values, got {len(vals)} in '{text}'"
        )
    return vals


def wait_for_csv(csv_path: Path, timeout_s: float, stable_for_s: float) -> None:
    start = time.time()
    last_size = -1
    stable_since = None

    while True:
        if csv_path.exists() and csv_path.is_file():
            size = csv_path.stat().st_size
            if size > 0 and size == last_size:
                if stable_since is None:
                    stable_since = time.time()
                elif (time.time() - stable_since) >= stable_for_s:
                    return
            else:
                stable_since = None
            last_size = size

        if timeout_s > 0.0 and (time.time() - start) > timeout_s:
            raise TimeoutError(f"Timed out waiting for stable CSV file: {csv_path}")
        time.sleep(0.2)


def load_rows(csv_path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        required = {
            "t",
            "active_joint",
            "excitation",
            "dq",
            "tau_measured",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV missing required columns: {sorted(missing)}")

        for r in reader:
            parsed = {k: parse_float(v) for k, v in r.items()}
            rows.append(parsed)
    return rows


def estimate_fs(t: np.ndarray) -> float:
    if t.size < 3:
        raise ValueError("Need at least 3 samples to estimate sampling frequency")
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 1e-9)]
    if dt.size < 2:
        raise ValueError("Unable to estimate dt from timestamps")
    return 1.0 / float(np.median(dt))


def butterworth_zero_phase(signal: np.ndarray, fs: float, fc_hz: float) -> np.ndarray:
    nyq = 0.5 * fs
    wn = fc_hz / nyq
    wn = min(max(wn, 1e-6), 0.999999)
    b, a = butter(4, wn, btype="low")

    # filtfilt requires enough samples; if too short, return original safely.
    min_len = 3 * max(len(a), len(b))
    if signal.size <= min_len:
        return signal.copy()
    return filtfilt(b, a, signal)


def sigmoid_friction(dq: np.ndarray, phi1: float, phi2: float, phi3: float) -> np.ndarray:
    phi2_eff = max(abs(phi2), 1e-6)
    x = np.clip(-phi2_eff * (dq + phi3), -60.0, 60.0)
    x0 = np.clip(-phi2_eff * phi3, -60.0, 60.0)
    term = phi1 / (1.0 + np.exp(x))
    term0 = phi1 / (1.0 + math.exp(x0))
    return term - term0


def fit_sigmoid_nelder_mead(
    dq: np.ndarray,
    residual: np.ndarray,
    phi1_max: float,
    phi2_max: float,
    phi3_abs_max: float,
) -> Tuple[float, float, float, float]:
    finite = np.isfinite(dq) & np.isfinite(residual)
    dq = dq[finite]
    residual = residual[finite]
    if dq.size < 20:
        raise ValueError("Need at least 20 valid samples for nonlinear fit")

    amp_guess = float(np.percentile(np.abs(residual), 90))
    amp_guess = min(max(amp_guess, 0.05), max(phi1_max, 0.05))
    starts = [
        np.array([amp_guess, min(10.0, phi2_max), 0.0]),
        np.array([-amp_guess, min(10.0, phi2_max), 0.0]),
        np.array([amp_guess * 0.7, min(6.0, phi2_max), 0.02]),
        np.array([-amp_guess * 0.7, min(6.0, phi2_max), 0.02]),
        np.array([amp_guess * 1.2, min(15.0, phi2_max), -0.02]),
        np.array([-amp_guess * 1.2, min(15.0, phi2_max), -0.02]),
    ]

    # Keep the fit in a physically plausible region to avoid unstable compensation.
    def bound_penalty(p: np.ndarray) -> float:
        p1, p2, p3 = float(p[0]), float(p[1]), float(p[2])
        pen = 0.0
        if p1 < -phi1_max:
            pen += (-phi1_max - p1) ** 2
        if p1 > phi1_max:
            pen += (p1 - phi1_max) ** 2
        if p2 < 1e-6:
            pen += (1e-6 - p2) ** 2
        if p2 > phi2_max:
            pen += (p2 - phi2_max) ** 2
        if abs(p3) > phi3_abs_max:
            pen += (abs(p3) - phi3_abs_max) ** 2
        return pen

    def objective(p: np.ndarray) -> float:
        pred = sigmoid_friction(dq, float(p[0]), float(p[1]), float(p[2]))
        err = residual - pred
        mse = float(np.mean(err * err))
        reg = 1e-6 * float(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])
        pen = 1e3 * bound_penalty(p)
        return mse + reg + pen

    best = None
    for x0 in starts:
        res = minimize(
            objective,
            x0,
            method="Nelder-Mead",
            options={"maxiter": 5000, "xatol": 1e-8, "fatol": 1e-10},
        )
        if best is None or res.fun < best.fun:
            best = res

    assert best is not None
    phi1, phi2, phi3 = [float(x) for x in best.x]
    phi1 = float(np.clip(phi1, -phi1_max, phi1_max))
    phi2 = float(np.clip(phi2, 1e-6, phi2_max))
    phi3 = float(np.clip(phi3, -phi3_abs_max, phi3_abs_max))
    pred = sigmoid_friction(dq, phi1, phi2, phi3)
    rmse = float(np.sqrt(np.mean((residual - pred) ** 2)))
    return phi1, phi2, phi3, rmse


def fit_piecewise_for_controller(
    dq: np.ndarray,
    residual: np.ndarray,
    hysteresis: float,
) -> Tuple[float, float, float, float]:
    pos = np.where(dq >= hysteresis)[0]
    neg = np.where(dq <= -hysteresis)[0]

    fvp = math.nan
    fcp = math.nan
    fvn = math.nan
    fcn = math.nan

    if pos.size >= 5:
        ap = np.column_stack([dq[pos], np.ones(pos.size)])
        xp, *_ = np.linalg.lstsq(ap, residual[pos], rcond=None)
        fvp = float(xp[0])
        fcp = float(xp[1])

    if neg.size >= 5:
        # controller uses tau = fvn*dq - fcn for negative branch
        an = np.column_stack([dq[neg], -np.ones(neg.size)])
        xn, *_ = np.linalg.lstsq(an, residual[neg], rcond=None)
        fvn = float(xn[0])
        fcn = float(xn[1])

    return fvp, fcp, fvn, fcn


def fit_model1_linear(dq: np.ndarray, residual: np.ndarray) -> Tuple[float, float, float]:
    finite = np.isfinite(dq) & np.isfinite(residual)
    dq_fit = dq[finite]
    r_fit = residual[finite]
    if dq_fit.size < 5:
        return math.nan, math.nan, math.nan

    a = np.column_stack([dq_fit, np.sign(dq_fit), np.ones(dq_fit.size)])
    x, *_ = np.linalg.lstsq(a, r_fit, rcond=None)
    return float(x[0]), float(x[1]), float(x[2])


def predict_model1_linear(dq: np.ndarray, fv: float, fc: float, fo: float) -> np.ndarray:
    if not (math.isfinite(fv) and math.isfinite(fc) and math.isfinite(fo)):
        return np.zeros_like(dq)
    return fv * dq + fc * np.sign(dq) + fo


def predict_model2_piecewise(
    dq: np.ndarray,
    fvp: float,
    fcp: float,
    fvn: float,
    fcn: float,
    hysteresis: float,
) -> np.ndarray:
    if not all(math.isfinite(v) for v in [fvp, fcp, fvn, fcn]):
        return np.zeros_like(dq)

    tau_pos = fvp * dq + fcp
    tau_neg = fvn * dq - fcn
    tau = np.empty_like(dq)

    pos = dq >= hysteresis
    neg = dq <= -hysteresis
    mid = ~(pos | neg)
    tau[pos] = tau_pos[pos]
    tau[neg] = tau_neg[neg]

    if np.any(mid):
        if hysteresis > 1e-9:
            alpha = np.clip((dq[mid] + hysteresis) / (2.0 * hysteresis), 0.0, 1.0)
            tau[mid] = alpha * tau_pos[mid] + (1.0 - alpha) * tau_neg[mid]
        else:
            tau[mid] = 0.5 * (tau_pos[mid] + tau_neg[mid])
    return tau


def safe_column(rows: Iterable[Dict[str, float]], key: str) -> np.ndarray:
    return np.array([r.get(key, float("nan")) for r in rows], dtype=float)


def process_joint(
    rows_joint: List[Dict[str, float]],
    fs_hz: float | None,
    fc_hz: float,
    hysteresis: float,
    diag_inertia: float,
    use_inertia: bool,
    phi1_max: float,
    phi2_max: float,
    phi3_abs_max: float,
) -> Tuple[JointFitResult, JointSignals]:
    rows_joint = sorted(rows_joint, key=lambda r: r.get("t", float("nan")))
    t = safe_column(rows_joint, "t")
    dq = safe_column(rows_joint, "dq")
    tau_meas = safe_column(rows_joint, "tau_measured")

    if fs_hz is None:
        fs = estimate_fs(t)
    else:
        fs = fs_hz

    tau_filt = butterworth_zero_phase(tau_meas, fs=fs, fc_hz=fc_hz)

    tau_cmd = safe_column(rows_joint, "tau_commanded")
    tau_task = safe_column(rows_joint, "tau_task")
    tau_g = safe_column(rows_joint, "tau_gravity")
    tau_c = safe_column(rows_joint, "tau_coriolis")
    tau_i_col = safe_column(rows_joint, "tau_inertia")
    ddq = np.gradient(dq, t, edge_order=1)
    tau_hat = np.zeros_like(tau_filt)

    cmd_ok = np.isfinite(tau_cmd)
    task_ok = np.isfinite(tau_task)
    g_ok = np.isfinite(tau_g)
    c_ok = np.isfinite(tau_c)
    i_ok = np.isfinite(tau_i_col)

    tau_gravity = np.where(g_ok, tau_g, 0.0)
    tau_coriolis = np.where(c_ok, tau_c, 0.0)
    tau_inertia = np.zeros_like(tau_filt)

    # For the article-style De Luca fit, prefer the full logged rigid-body model
    # so the residual matches Delta tau = tau - tau_hat from inverse dynamics.
    if np.mean((task_ok & c_ok & g_ok).astype(float)) > 0.8:
        tau_hat = (
            np.where(task_ok, tau_task, 0.0)
            + np.where(c_ok, tau_c, 0.0)
            + np.where(g_ok, tau_g, 0.0)
        )
        if np.any(i_ok):
            tau_inertia = np.where(i_ok, tau_i_col, 0.0)
            tau_hat += tau_inertia
        elif use_inertia:
            tau_inertia = diag_inertia * ddq
            tau_hat += tau_inertia
    # Fallback for logs that only expose the commanded torque.
    elif np.mean(cmd_ok.astype(float)) > 0.8:
        tau_hat_cmd = np.where(cmd_ok, tau_cmd, 0.0)
        tau_hat = tau_hat_cmd
        if np.mean(g_ok.astype(float)) > 0.8:
            r_cmd = tau_filt - tau_hat_cmd
            r_cmd_g = tau_filt - (tau_hat_cmd + np.where(g_ok, tau_g, 0.0))
            std_cmd = float(np.nanstd(r_cmd))
            std_cmd_g = float(np.nanstd(r_cmd_g))
            if np.isfinite(std_cmd) and np.isfinite(std_cmd_g) and std_cmd_g < 0.9 * std_cmd:
                tau_hat = tau_hat_cmd + np.where(g_ok, tau_g, 0.0)
    # Legacy fallback for older logs.
    else:
        tau_hat += np.where(g_ok, tau_g, 0.0)
        tau_hat += np.where(c_ok, tau_c, 0.0)
        if use_inertia:
            if np.any(i_ok):
                tau_inertia = np.where(i_ok, tau_i_col, 0.0)
            else:
                tau_inertia = diag_inertia * ddq
            tau_hat += tau_inertia

    residual = tau_filt - tau_hat
    phi1, phi2, phi3, rmse = fit_sigmoid_nelder_mead(
        dq,
        residual,
        phi1_max=phi1_max,
        phi2_max=phi2_max,
        phi3_abs_max=phi3_abs_max,
    )

    fvp, fcp, fvn, fcn = fit_piecewise_for_controller(dq, residual, hysteresis)
    tau_friction = sigmoid_friction(dq, phi1, phi2, phi3)
    tau_compensated = tau_meas - tau_friction

    fv1, fc1, fo1 = fit_model1_linear(dq, residual)
    tau_friction_model1 = predict_model1_linear(dq, fv1, fc1, fo1)
    tau_compensated_model1 = tau_meas - tau_friction_model1

    tau_friction_model2 = predict_model2_piecewise(dq, fvp, fcp, fvn, fcn, hysteresis)
    tau_compensated_model2 = tau_meas - tau_friction_model2

    joint_id = int(round(rows_joint[0]["active_joint"]))
    result = JointFitResult(
        joint=joint_id,
        count=int(len(rows_joint)),
        phi1=phi1,
        phi2=phi2,
        phi3=phi3,
        rmse=rmse,
        fvp=fvp,
        fcp=fcp,
        fvn=fvn,
        fcn=fcn,
    )

    signals = JointSignals(
        t=t,
        dq=dq,
        tau_raw=tau_meas,
        tau_filt=tau_filt,
        tau_hat=tau_hat,
        residual=residual,
        tau_compensated=tau_compensated,
        tau_gravity=tau_gravity,
        tau_inertia=tau_inertia,
        tau_coriolis=tau_coriolis,
        tau_friction=tau_friction,
        tau_friction_model1=tau_friction_model1,
        tau_friction_model2=tau_friction_model2,
        tau_compensated_model1=tau_compensated_model1,
        tau_compensated_model2=tau_compensated_model2,
    )

    return result, signals


def plot_joint_model_comparison_grid(
    path: Path,
    joint_signals: Dict[int, JointSignals],
    num_joints: int,
) -> None:
    if plt is None:
        raise RuntimeError(
            "matplotlib is required for plotting. Install with: pip install matplotlib"
        )

    fig, axes = plt.subplots(
        num_joints,
        3,
        figsize=(18.5, max(2.6 * num_joints, 8.0)),
        constrained_layout=False,
    )
    if num_joints == 1:
        axes = np.array([axes])

    col_titles = ["De Luca (sigmoid)", "Model 1 (fv, fc, fo)", "Model 2 (hysteresis)"]
    for c in range(3):
        axes[0, c].set_title(col_titles[c])

    for row_idx in range(num_joints):
        joint = row_idx + 1
        sig = joint_signals.get(joint)
        for col_idx in range(3):
            ax = axes[row_idx, col_idx]
            if sig is None:
                ax.text(0.5, 0.5, f"Joint {joint}: no data", ha="center", va="center", transform=ax.transAxes)
                ax.set_xticks([])
                ax.set_yticks([])
                continue

            t_rel = sig.t - sig.t[0]
            if col_idx == 0:
                tau_comp = sig.tau_compensated
            elif col_idx == 1:
                tau_comp = sig.tau_compensated_model1
            else:
                tau_comp = sig.tau_compensated_model2
            tau_raw = sig.tau_raw

            ax.fill_between(
                t_rel,
                tau_raw,
                tau_comp,
                color="tab:red",
                alpha=0.18,
                label="Compensation difference",
            )
            ax.plot(t_rel, tau_raw, color="black", linewidth=1.1, label="Raw torque")
            ax.plot(t_rel, tau_comp, color="tab:blue", linewidth=1.1, label="Compensated torque")
            ax.grid(True, alpha=0.25)
            ax.margins(x=0.01, y=0.08)

            if col_idx == 0:
                ax.set_ylabel(f"J{joint} [Nm]")
            if row_idx == num_joints - 1:
                ax.set_xlabel("Time [s]")

    path.parent.mkdir(parents=True, exist_ok=True)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.992),
        ncol=3,
        frameon=False,
        columnspacing=2.0,
        handlelength=2.8,
    )
    fig.subplots_adjust(left=0.06, right=0.995, bottom=0.045, top=0.94, hspace=0.42, wspace=0.2)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def write_model2_csv(path: Path, results: List[JointFitResult], hysteresis: float, num_joints: int) -> None:
    by_joint = {r.joint: r for r in results}
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["joint", "hysteresis", "fv_p", "fc_p", "fv_n", "fc_n"])
        for j in range(1, num_joints + 1):
            r = by_joint.get(j)
            if r is None:
                w.writerow([j, hysteresis, math.nan, math.nan, math.nan, math.nan])
            else:
                w.writerow([j, hysteresis, r.fvp, r.fcp, r.fvn, r.fcn])


def write_sigmoid_csv(path: Path, results: List[JointFitResult], num_joints: int) -> None:
    by_joint = {r.joint: r for r in results}
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["joint", "phi1", "phi2", "phi3", "rmse", "samples"])
        for j in range(1, num_joints + 1):
            r = by_joint.get(j)
            if r is None:
                w.writerow([j, math.nan, math.nan, math.nan, math.nan, 0])
            else:
                w.writerow([j, r.phi1, r.phi2, r.phi3, r.rmse, r.count])


def write_residual_samples(path: Path, rows: List[Tuple[int, float, float, float, float, float]]) -> None:
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["joint", "t", "dq", "tau_filt", "tau_hat", "tau_residual"])
        w.writerows(rows)


def write_robot_config_snippet(
    path: Path,
    results: List[JointFitResult],
    hysteresis: float,
    num_joints: int,
    profile: str,
    robot_name: str,
    friction_model: str,
) -> None:
    by_joint = {r.joint: r for r in results}
    phi1 = []
    phi2 = []
    phi3 = []
    fvp = []
    fcp = []
    fvn = []
    fcn = []
    hyst = []

    for j in range(1, num_joints + 1):
        r = by_joint.get(j)
        phi1.append(math.nan if r is None else r.phi1)
        phi2.append(math.nan if r is None else r.phi2)
        phi3.append(math.nan if r is None else r.phi3)
        hyst.append(hysteresis)
        fvp.append(math.nan if r is None else r.fvp)
        fcp.append(math.nan if r is None else r.fcp)
        fvn.append(math.nan if r is None else r.fvn)
        fcn.append(math.nan if r is None else r.fcn)

    def fmt_vec(v: List[float]) -> str:
        # ROS parameter arrays must contain numeric scalars; avoid NaN tokens.
        # Emit fixed-point floats so the YAML is consistently parsed as double[].
        return "[" + ", ".join(f"{x:.9f}" if math.isfinite(x) else "0.000000000" for x in v) + "]"

    text = (
        f"{profile}:\n"
        f"  robot_name: {robot_name}\n"
        f"\n"
        f"  cartesian_impedance_controller:\n"
        f"    ros__parameters:\n"
        f"      add_friction_compensation: true\n"
        f"      friction_compensation:\n"
        f"        model: {friction_model}\n"
        f"        scale: 1.0\n"
        f"        phi1: {fmt_vec(phi1)}\n"
        f"        phi2: {fmt_vec(phi2)}\n"
        f"        phi3: {fmt_vec(phi3)}\n"
        f"        hyst: {fmt_vec(hyst)}\n"
        f"        fvp: {fmt_vec(fvp)}\n"
        f"        fcp: {fmt_vec(fcp)}\n"
        f"        fvn: {fmt_vec(fvn)}\n"
        f"        fcn: {fmt_vec(fcn)}\n"
    )
    path.write_text(text)


def main() -> int:
    parser = argparse.ArgumentParser(description="Post-process friction_estimation_v2 CSV")
    parser.add_argument(
        "--csv",
        default="/tmp/friction_estimation_measurements_v2.csv",
        help="Input CSV path produced by FrictionEstimationImplV2",
    )
    parser.add_argument(
        "--wait",
        action="store_true",
        help="Wait until CSV exists and file size is stable before processing",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=1800.0,
        help="Timeout in seconds for --wait (<=0 means no timeout)",
    )
    parser.add_argument(
        "--wait-stable-for",
        type=float,
        default=1.0,
        help="File-size stability duration in seconds for --wait",
    )
    parser.add_argument("--fs", type=float, default=0.0, help="Sampling frequency [Hz]. 0 = estimate from timestamps")
    parser.add_argument(
        "--fc",
        type=float,
        default=3.0,
        help="Butterworth cutoff frequency [Hz] used for model fitting",
    )
    parser.add_argument("--hysteresis", type=float, default=0.05, help="Velocity hysteresis threshold for controller piecewise fit")
    parser.add_argument(
        "--diag-inertia",
        type=str,
        default="",
        help="Optional 7-value comma-separated diagonal inertia approximation [Nm/(rad/s^2)]",
    )
    parser.add_argument(
        "--use-inertia",
        action="store_true",
        help="Include inertia contribution when reconstructing tau_hat from components.",
    )
    parser.add_argument("--num-joints", type=int, default=7)
    parser.add_argument("--profile", default="ROBOT_1")
    parser.add_argument("--robot-name", default="fr3")
    parser.add_argument(
        "--friction-model",
        choices=["hysteresis", "sigmoid", "auto"],
        default="hysteresis",
        help="Friction compensation model name emitted into the YAML snippet",
    )
    parser.add_argument("--model2-output", default="/tmp/model2_estimated_friction.csv")
    parser.add_argument("--sigmoid-output", default="/tmp/model2_sigmoid_estimated_friction.csv")
    parser.add_argument("--residual-output", default="/tmp/friction_residual_estimation_v2.csv")
    parser.add_argument("--yaml-output", default="/tmp/robot_config.updated.yaml")
    parser.add_argument(
        "--plot-dir",
        default="/tmp/friction_estimation_v2_plots",
        help="Directory for generated visualization PNGs",
    )
    parser.add_argument(
        "--skip-plots",
        action="store_true",
        help="Skip generating torque visualizations",
    )
    parser.add_argument(
        "--phi1-max",
        type=float,
        default=5.0,
        help="Absolute bound for phi1 during fit",
    )
    parser.add_argument("--phi2-max", type=float, default=30.0, help="Upper bound for phi2 during fit")
    parser.add_argument("--phi3-abs-max", type=float, default=0.2, help="Absolute bound for phi3 during fit")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if args.wait:
        wait_for_csv(csv_path, timeout_s=args.wait_timeout, stable_for_s=args.wait_stable_for)

    rows = load_rows(csv_path)
    rows = [r for r in rows if int(round(r.get("excitation", 0.0))) == 1]
    if not rows:
        raise ValueError("No excitation rows found in CSV")

    diag_inertia = parse_vector(args.diag_inertia, expected_len=args.num_joints)
    diag_default = [0.0] * args.num_joints if not diag_inertia else diag_inertia

    rows_by_joint: Dict[int, List[Dict[str, float]]] = {}
    for r in rows:
        j = int(round(r.get("active_joint", float("nan"))))
        if 1 <= j <= args.num_joints:
            rows_by_joint.setdefault(j, []).append(r)

    fs_hz = None if args.fs <= 0.0 else args.fs
    fit_results: List[JointFitResult] = []
    residual_rows: List[Tuple[int, float, float, float, float, float]] = []
    generated_plots: List[Path] = []
    joint_signals: Dict[int, JointSignals] = {}
    plot_dir = Path(args.plot_dir)
    if not args.skip_plots:
        plot_dir.mkdir(parents=True, exist_ok=True)

    for joint in sorted(rows_by_joint.keys()):
        result, signals = process_joint(
            rows_by_joint[joint],
            fs_hz=fs_hz,
            fc_hz=args.fc,
            hysteresis=args.hysteresis,
            diag_inertia=diag_default[joint - 1],
            use_inertia=args.use_inertia,
            phi1_max=args.phi1_max,
            phi2_max=args.phi2_max,
            phi3_abs_max=args.phi3_abs_max,
        )
        fit_results.append(result)
        joint_signals[joint] = signals
        residual_rows.extend(
            (joint, float(ti), float(dqi), float(tf), float(th), float(tr))
            for ti, dqi, tf, th, tr in zip(
                signals.t,
                signals.dq,
                signals.tau_filt,
                signals.tau_hat,
                signals.residual,
            )
        )
    if not args.skip_plots:
        plot_path = plot_dir / "torque_model_comparison_grid.png"
        plot_joint_model_comparison_grid(
            plot_path,
            joint_signals=joint_signals,
            num_joints=args.num_joints,
        )
        generated_plots.append(plot_path)

    model2_out = Path(args.model2_output)
    sigmoid_out = Path(args.sigmoid_output)
    residual_out = Path(args.residual_output)
    yaml_out = Path(args.yaml_output)

    model2_out.parent.mkdir(parents=True, exist_ok=True)
    sigmoid_out.parent.mkdir(parents=True, exist_ok=True)
    residual_out.parent.mkdir(parents=True, exist_ok=True)
    yaml_out.parent.mkdir(parents=True, exist_ok=True)

    write_model2_csv(model2_out, fit_results, args.hysteresis, args.num_joints)
    write_sigmoid_csv(sigmoid_out, fit_results, args.num_joints)
    write_residual_samples(residual_out, residual_rows)
    write_robot_config_snippet(
        yaml_out,
        fit_results,
        args.hysteresis,
        args.num_joints,
        args.profile,
        args.robot_name,
        args.friction_model,
    )

    print(f"Processed joints: {','.join(str(r.joint) for r in fit_results)}")
    print(f"Model2 CSV: {model2_out}")
    print(f"Sigmoid CSV: {sigmoid_out}")
    print(f"Residual CSV: {residual_out}")
    print(f"Robot config snippet: {yaml_out}")
    if not args.skip_plots:
        print(f"Torque plots dir: {plot_dir}")
        if generated_plots:
            print("Generated plots:")
            for p in generated_plots:
                print(f"  - {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
