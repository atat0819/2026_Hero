# -*- coding: utf-8 -*-
"""Fit the chassis power model used by RtosTask/can_send_task.cpp.

Firmware model, evaluated once for each motor and then summed:
    P = K0 + K1*I + K2*abs(w) + K3*I*w + K4*I*I + K5*w*w

Supported VOFA exports:
    calibration9: P, w1, I1, w2, I2, w3, I3, w4, I4
    comparison10: P, P_est, w1, I1, w2, I2, w3, I3, w4, I4

The script deliberately has no sklearn/scipy dependency.  It uses a robust
Huber regression with ridge regularization, then converts the result back to
the six coefficients consumed directly by the firmware.
"""

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

CSV_PATH = r"E:\power_800.csv"
NUM_MOTORS = 4

# "auto" accepts exactly 9 or 10 VOFA data columns.  Set this explicitly when
# an export contains extra columns such as a timestamp.
INPUT_LAYOUT = "auto"       # "auto", "calibration9", or "comparison10"
POWER_COLUMN = 0
ESTIMATE_COLUMN = None       # None = inferred from INPUT_LAYOUT
FIRST_MOTOR_COLUMN = None    # None = inferred from INPUT_LAYOUT

DROP_HARD_LIMITS = True
I_LIMIT = 25.0               # A
W_LIMIT = 950.0              # rotor rad/s
P_LIMIT = 2000.0             # complete chassis W

# Keep disabled unless every logged channel has the same known low-pass delay.
# If enabled, filtering is causal: it never uses future samples.
CAUSAL_FILTER_WINDOW = 0

VAL_FRACTION = 0.20          # final contiguous block; never used for tuning
MAX_FEATURE_LAG_SAMPLES = 60 # search P[t] against motor features at t-lag
LAG_SEARCH_FRACTION = 0.80   # only this prefix is used to choose the lag

# alpha is scaled by the number of samples, making the setting stable when the
# logging duration changes.  alpha is selected by blocked CV on training data.
RIDGE_ALPHAS = (0.0, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1)
CV_FOLDS = 4
HUBER_DELTA = 1.5
HUBER_MAX_ITER = 40
HUBER_TOL = 1e-8


def load_data(path):
    if not os.path.exists(path):
        sys.exit(f"CSV file does not exist: {path}")
    df = pd.read_csv(path)
    if df.empty:
        sys.exit("CSV has no data rows.")
    print(f"Loaded: {path}")
    print(f"Rows: {len(df)}, columns: {df.shape[1]}")
    return df


def resolve_layout(column_count):
    """Return (power column, firmware-estimate column, first motor column)."""
    layout = INPUT_LAYOUT.lower()
    if layout == "auto":
        if column_count == 1 + 2 * NUM_MOTORS:
            layout = "calibration9"
        elif column_count == 2 + 2 * NUM_MOTORS:
            layout = "comparison10"
        else:
            sys.exit(
                "Cannot infer CSV layout. Expected 9 columns for "
                "[P,w1,I1,...] or 10 columns for [P,P_est,w1,I1,...]. "
                "Set INPUT_LAYOUT and the column constants for this export."
            )

    if layout == "calibration9":
        default_est, default_first = None, 1
    elif layout == "comparison10":
        default_est, default_first = 1, 2
    else:
        sys.exit("INPUT_LAYOUT must be auto, calibration9, or comparison10.")

    est = default_est if ESTIMATE_COLUMN is None else ESTIMATE_COLUMN
    first = default_first if FIRST_MOTOR_COLUMN is None else FIRST_MOTOR_COLUMN
    needed = first + 2 * NUM_MOTORS
    if column_count < needed:
        sys.exit(f"Layout needs at least {needed} columns, CSV has {column_count}.")
    print(f"Layout: {layout}; P={POWER_COLUMN}, P_est={est}, w1={first}, I1={first + 1}")
    return POWER_COLUMN, est, first


def numeric_column(df, index):
    return pd.to_numeric(df.iloc[:, index], errors="coerce").to_numpy(dtype=float)


def extract_data(df):
    p_col, est_col, first = resolve_layout(df.shape[1])
    power = numeric_column(df, p_col)
    speed = np.column_stack([numeric_column(df, first + 2 * motor)
                             for motor in range(NUM_MOTORS)])
    current = np.column_stack([numeric_column(df, first + 2 * motor + 1)
                               for motor in range(NUM_MOTORS)])
    estimate = numeric_column(df, est_col) if est_col is not None else None
    return power, speed, current, estimate


def clean_data(power, speed, current, estimate):
    mask = np.isfinite(power) & np.isfinite(speed).all(axis=1) & np.isfinite(current).all(axis=1)
    if estimate is not None:
        # P_est is diagnostic only, therefore a missing P_est must not discard
        # an otherwise valid fitting sample.
        estimate = estimate.copy()
    if DROP_HARD_LIMITS:
        mask &= (np.abs(current) <= I_LIMIT).all(axis=1)
        mask &= (np.abs(speed) <= W_LIMIT).all(axis=1)
        mask &= np.abs(power) <= P_LIMIT

    removed = len(power) - int(mask.sum())
    power, speed, current = power[mask], speed[mask], current[mask]
    estimate = estimate[mask] if estimate is not None else None
    print(f"Valid samples: {len(power)}; removed: {removed}")

    if CAUSAL_FILTER_WINDOW > 1:
        window = CAUSAL_FILTER_WINDOW
        power = pd.Series(power).rolling(window, min_periods=1).mean().to_numpy()
        for motor in range(NUM_MOTORS):
            speed[:, motor] = pd.Series(speed[:, motor]).rolling(window, min_periods=1).mean().to_numpy()
            current[:, motor] = pd.Series(current[:, motor]).rolling(window, min_periods=1).mean().to_numpy()
        if estimate is not None:
            estimate = pd.Series(estimate).rolling(window, min_periods=1).mean().to_numpy()
        print(f"Applied causal moving average, window={window}")
    return power, speed, current, estimate


def build_features(speed, current):
    """Exactly the same summed features as post_power in can_send_task.cpp."""
    return np.column_stack((
        np.full(len(current), NUM_MOTORS, dtype=float),
        current.sum(axis=1),
        np.abs(speed).sum(axis=1),
        (current * speed).sum(axis=1),
        np.square(current).sum(axis=1),
        np.square(speed).sum(axis=1),
    ))


def predict(coefficients, speed, current):
    return build_features(speed, current) @ coefficients


def align_feature_lag(power, speed, current, estimate, lag):
    """Align P[t] with features[t-lag]. Positive lag means P is delayed."""
    if lag > 0:
        return power[lag:], speed[:-lag], current[:-lag], (estimate[lag:] if estimate is not None else None)
    if lag < 0:
        return power[:lag], speed[-lag:], current[-lag:], (estimate[:lag] if estimate is not None else None)
    return power, speed, current, estimate


def standardized_design(features):
    """Keep a unit intercept and standardize the five nonconstant features."""
    mean = features[:, 1:].mean(axis=0)
    scale = features[:, 1:].std(axis=0)
    scale[scale < 1e-12] = 1.0
    design = np.empty_like(features)
    design[:, 0] = 1.0
    design[:, 1:] = (features[:, 1:] - mean) / scale
    return design, mean, scale


def coefficients_from_standardized(beta, mean, scale):
    coefficients = np.empty(6, dtype=float)
    coefficients[1:] = beta[1:] / scale
    # The firmware intercept feature is NUM_MOTORS, not a literal 1.
    coefficients[0] = (beta[0] - np.dot(coefficients[1:], mean)) / NUM_MOTORS
    return coefficients


def fit_huber_ridge(power, speed, current, alpha):
    """Robust ridge regression; alpha regularizes only nonconstant terms."""
    features = build_features(speed, current)
    design, mean, scale = standardized_design(features)
    penalty = np.diag([0.0] + [alpha * len(power)] * 5)
    weights = np.ones(len(power), dtype=float)
    beta = np.zeros(6, dtype=float)

    for _ in range(HUBER_MAX_ITER):
        weighted_design = design * weights[:, None]
        lhs = design.T @ weighted_design + penalty
        rhs = design.T @ (weights * power)
        try:
            next_beta = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            next_beta = np.linalg.lstsq(lhs, rhs, rcond=None)[0]

        residual = power - design @ next_beta
        mad = np.median(np.abs(residual - np.median(residual)))
        robust_scale = max(1.4826 * mad, 1e-6)
        cutoff = HUBER_DELTA * robust_scale
        next_weights = np.minimum(1.0, cutoff / np.maximum(np.abs(residual), 1e-12))
        if np.max(np.abs(next_beta - beta)) < HUBER_TOL:
            beta = next_beta
            break
        beta, weights = next_beta, next_weights
    return coefficients_from_standardized(beta, mean, scale)


def rmse(actual, predicted):
    return float(np.sqrt(np.mean(np.square(actual - predicted))))


def select_feature_lag(power, speed, current):
    """Choose timing only from a prefix which excludes the final holdout."""
    search_end = max(1000, int(len(power) * LAG_SEARCH_FRACTION))
    search_end = min(search_end, len(power))
    p0, w0, i0 = power[:search_end], speed[:search_end], current[:search_end]
    split = int(len(p0) * 0.75)
    best_lag, best_score = 0, float("inf")

    for lag in range(-MAX_FEATURE_LAG_SAMPLES, MAX_FEATURE_LAG_SAMPLES + 1):
        p, w, i, _ = align_feature_lag(p0, w0, i0, None, lag)
        if len(p) < 200:
            continue
        cut = min(split, len(p) - 100)
        coefficients = fit_huber_ridge(p[:cut], w[:cut], i[:cut], alpha=1e-3)
        score = rmse(p[cut:], predict(coefficients, w[cut:], i[cut:]))
        if score < best_score:
            best_lag, best_score = lag, score
    print(f"Selected feature lag: {best_lag:+d} samples; tuning RMSE={best_score:.3f} W")
    return best_lag


def select_alpha(power, speed, current):
    """Blocked CV prevents tuning only on an adjacent, nearly identical run."""
    folds = min(CV_FOLDS, max(2, len(power) // 500))
    boundaries = np.linspace(0, len(power), folds + 1, dtype=int)
    scores = []
    for alpha in RIDGE_ALPHAS:
        squared_error, count = 0.0, 0
        for fold in range(folds):
            start, end = boundaries[fold], boundaries[fold + 1]
            train_mask = np.ones(len(power), dtype=bool)
            train_mask[start:end] = False
            coefficients = fit_huber_ridge(power[train_mask], speed[train_mask], current[train_mask], alpha)
            residual = power[start:end] - predict(coefficients, speed[start:end], current[start:end])
            squared_error += float(residual @ residual)
            count += len(residual)
        score = float(np.sqrt(squared_error / count))
        scores.append(score)
        print(f"CV alpha={alpha:.5g}: RMSE={score:.3f} W")
    index = int(np.argmin(scores))
    print(f"Selected ridge alpha: {RIDGE_ALPHAS[index]:.5g}")
    return RIDGE_ALPHAS[index]


def metrics(actual, predicted):
    error = actual - predicted
    ss_total = np.sum(np.square(actual - actual.mean()))
    r2 = 1.0 - np.sum(np.square(error)) / ss_total if ss_total > 0 else float("nan")
    under = np.maximum(error, 0.0)
    return {
        "r2": r2,
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "mae": float(np.mean(np.abs(error))),
        "abs_p90": float(np.quantile(np.abs(error), 0.90)),
        "under_p95": float(np.quantile(under, 0.95)),
        "under_max": float(np.max(under)),
    }


def print_metrics(name, actual, predicted):
    result = metrics(actual, predicted)
    print(
        f"{name:18s} R2={result['r2']:7.4f}  RMSE={result['rmse']:7.3f} W  "
        f"MAE={result['mae']:7.3f} W  |e|P90={result['abs_p90']:7.3f} W  "
        f"under-P95={result['under_p95']:7.3f} W"
    )
    return result


def diagnostics(power, speed, current):
    print("\nInput ranges")
    print(f"P: {power.min():.3f} .. {power.max():.3f} W")
    for motor in range(NUM_MOTORS):
        print(
            f"M{motor + 1}: w={speed[:, motor].min():.1f} .. {speed[:, motor].max():.1f} rad/s, "
            f"I={current[:, motor].min():.2f} .. {current[:, motor].max():.2f} A"
        )
    design, _, _ = standardized_design(build_features(speed, current))
    condition = np.linalg.cond(design)
    print(f"Standardized feature condition number: {condition:.2e}")
    if condition > 1e4:
        print("WARNING: features are strongly correlated; collect more independent speed/current conditions.")


def plot_results(path, power, speed, current, estimate, prediction, train_count, lag, coefficients):
    index = np.arange(len(power))
    error = power - prediction
    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=False)

    axes[0].plot(index, power, color="black", linewidth=0.7, label="measured P")
    axes[0].plot(index, prediction, color="tab:red", linewidth=0.7, label="fitted P")
    if estimate is not None and np.isfinite(estimate).any():
        axes[0].plot(index, estimate, color="tab:green", linewidth=0.6, alpha=0.8, label="firmware P_est")
    axes[0].axvspan(train_count, len(power), color="orange", alpha=0.15, label="holdout")
    axes[0].set_ylabel("power (W)")
    axes[0].set_title(f"Chassis power fit, feature lag={lag:+d} samples")
    axes[0].legend(loc="upper right")
    axes[0].grid(alpha=0.25)

    axes[1].plot(index, error, color="tab:blue", linewidth=0.6)
    axes[1].axhline(0.0, color="black", linewidth=0.6)
    axes[1].axvspan(train_count, len(power), color="orange", alpha=0.15)
    axes[1].set_ylabel("P_measured - P_fit (W)")
    axes[1].grid(alpha=0.25)

    axes[2].scatter(speed.reshape(-1), current.reshape(-1), s=1.5, alpha=0.35)
    axes[2].set_xlabel("rotor speed (rad/s)")
    axes[2].set_ylabel("current (A)")
    axes[2].set_title("Per-motor operating-point coverage")
    axes[2].grid(alpha=0.25)

    fig.tight_layout()
    output_dir = os.path.dirname(os.path.abspath(path))
    stem = os.path.splitext(os.path.basename(path))[0]
    output = os.path.join(output_dir, f"{stem}_power_predict_fit.png")
    fig.savefig(output, dpi=160)
    plt.close(fig)
    print(f"Plot: {output}")

    report = pd.DataFrame({
        "sample": index,
        "power_measured_w": power,
        "power_predicted_w": prediction,
        "residual_w": error,
    })
    if estimate is not None:
        report["firmware_estimate_w"] = estimate
    report_path = os.path.join(output_dir, f"{stem}_power_predict_report.csv")
    report.to_csv(report_path, index=False)
    print(f"Per-sample report: {report_path}")


def main(path):
    df = load_data(path)
    power, speed, current, estimate = extract_data(df)
    power, speed, current, estimate = clean_data(power, speed, current, estimate)
    if len(power) < 1000:
        sys.exit(f"Need at least 1000 valid samples; only {len(power)} remain.")
    diagnostics(power, speed, current)

    lag = select_feature_lag(power, speed, current)
    power, speed, current, estimate = align_feature_lag(power, speed, current, estimate, lag)
    train_count = int(len(power) * (1.0 - VAL_FRACTION))
    if train_count < 500 or len(power) - train_count < 100:
        sys.exit("Not enough data after lag alignment for train/holdout split.")

    p_train, w_train, i_train = power[:train_count], speed[:train_count], current[:train_count]
    p_valid, w_valid, i_valid = power[train_count:], speed[train_count:], current[train_count:]
    alpha = select_alpha(p_train, w_train, i_train)
    coefficients = fit_huber_ridge(p_train, w_train, i_train, alpha)
    prediction = predict(coefficients, speed, current)

    print("\nFit quality")
    print_metrics("train", p_train, prediction[:train_count])
    print_metrics("holdout", p_valid, prediction[train_count:])
    if estimate is not None and np.isfinite(estimate[train_count:]).any():
        finite = np.isfinite(estimate[train_count:])
        print_metrics("firmware holdout", p_valid[finite], estimate[train_count:][finite])
        estimate_valid = estimate[train_count:][finite]
        prediction_valid = prediction[train_count:][finite]
        mismatch = np.sqrt(np.mean(np.square(estimate_valid - prediction_valid)))
        print(f"Firmware/fitted prediction difference: {mismatch:.3f} W")
        print("NOTE: a large difference is expected if VOFA logs getCurrent() while post_power uses motor_output.")

    print("\nCoefficients for can_send_task.cpp")
    for name, value in zip(("K0", "K1", "K2", "K3", "K4", "K5"), coefficients):
        print(f"{name} = {value:.9f}f")
    print("CorrectionConstant = 0.0f")
    print("// P_total = sum_motor(K0 + K1*I + K2*abs(w) + K3*I*w + K4*I*I + K5*w*w)")

    plot_results(path, power, speed, current, estimate, prediction, train_count, lag, coefficients)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else CSV_PATH)
