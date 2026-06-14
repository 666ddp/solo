import argparse
import csv
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import serial

try:
    from skopt import gp_minimize
    from skopt.space import Real
except Exception:
    gp_minimize = None
    Real = None

DSP_PORT = "COM3"
BAUD_RATE = 115200
DT = 0.001
RECORD_LENGTH = 1000
STEP_RECORD_LENGTH = 4000
STEP1_END_INDEX = 1000
STEP2_END_INDEX = 2500
TARGET_RPM = 80.0
COOLDOWN = 10.0
ACK_TIMEOUT = 6.0
DATA_TIMEOUT = 15.0
STEP_DATA_TIMEOUT = 30.0
POST_DATA_READY_TIMEOUT = 3.0
PRE_SEND_QUIET = 1.5
FAIL_SCORE = 200000.0
MIN_VALID_RPM = TARGET_RPM * 0.75
TAIL_FRACTION = 0.50
STARTUP_IGNORE_TIME = 0.0
RISE_FRACTION = 0.90
SETTLING_BAND = 0.08
SMOOTHING_WINDOW = 25
SPIKE_MAX_ABS_RPM = 300.0
SPIKE_MAX_STEP_RPM = 150.0
OSC_EVAL_FRACTION = 0.90
OSC_STD_WEIGHT = 120.0
OSC_RANGE_WEIGHT = 20.0
OSC_MEAN_ERROR_WEIGHT = 35.0
CURRENT_WARN_A = 4.5
CURRENT_RMS_WARN_A = 3.0
CURRENT_HARD_A = 5.8
STALL_CURRENT_A = 5.2
STALL_TAIL_RPM = TARGET_RPM * 0.25

BASE_DIR = Path(__file__).resolve().parent / "solo_axis123_results"
BEST_PARAM_DIR = BASE_DIR / "best_params"

PTSMC_DEFAULT = [
    0.94834, 0.42657, 142.38, 0.59725, 91.208, 315.0,
    0.7674, 0.93304, 51.429, 26.385, 275.39,
]
PI_DEFAULT = [20.05826580, 0.00090424]
FTSMC_DEFAULT = [583.6, 558.9, 79.1, 4.0, 10.0]

FAULT_LINES = {"RUN_FAIL", "NO_DC", "HALL_FAIL", "STALL", "DC_DROP", "USER_STOP"}

CONTROLLERS = {
    "ptsmc": {
        "name": "PTSMC控制器",
        "label": "PTSMC控制器",
        "bo_cmd": "P",
        "step_cmd": "STEP1",
        "high_step_cmd": "HSTEP1",
        "hold_cmd": "HOLD1",
        "params": PTSMC_DEFAULT,
        "param_names": ["P1", "P2", "A11", "B11", "A22", "B22", "C3", "P3", "A33", "B33", "C8"],
        "space": [(0.75, 0.999), (0.30, 0.99), (80.0, 160.0), (0.001, 1.0), (60.0, 130.0), (300.0, 450.0), (0.40, 1.20), (0.90, 0.999), (40.0, 120.0), (1.0, 40.0), (250.0, 700.0)],
    },
    "pi": {
        "name": "PI控制器",
        "label": "PI控制器",
        "bo_cmd": "PI2",
        "step_cmd": "STEP2",
        "high_step_cmd": "HSTEP2",
        "hold_cmd": "HOLD2",
        "params": PI_DEFAULT,
        "param_names": ["Kp", "Ki"],
        "space": [(17.0, 22.0), (0.0009, 0.0015)],
    },
    "ftsmc": {
        "name": "FTSMC控制器",
        "label": "FTSMC控制器",
        "bo_cmd": "FT3",
        "step_cmd": "STEP3",
        "high_step_cmd": "HSTEP3",
        "hold_cmd": "HOLD3",
        "params": FTSMC_DEFAULT,
        "param_names": ["K1", "K2", "MU", "DO_K", "DO_I"],
        "space": [(580.0, 590.0), (530.0, 575.0), (70.0, 90.0), (1.0, 8.0), (1.0, 20.0)],
    },
}

CONTROLLER_ALIASES = {
    "axis1": "ptsmc",
    "axis2": "pi",
    "axis3": "ftsmc",
    "ptsmc": "ptsmc",
    "pi": "pi",
    "ftsmc": "ftsmc",
}


def normalize_controller(key):
    return CONTROLLER_ALIASES[key.lower()]


def setup_font():
    for font in [r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\simsun.ttc"]:
        p = Path(font)
        if p.exists():
            font_manager.fontManager.addfont(str(p))
            name = font_manager.FontProperties(fname=str(p)).get_name()
            plt.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            plt.rcParams["font.family"] = "sans-serif"
            break
    plt.rcParams["axes.unicode_minus"] = False


def read_line_until(ser, deadline):
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="ignore").strip()
        if line:
            return line
    return None


def send_stop(ser):
    ser.reset_input_buffer()
    ser.write(b"STOP\n")
    ser.flush()
    deadline = time.time() + ACK_TIMEOUT
    while time.time() < deadline:
        line = read_line_until(ser, deadline)
        if line in {"STOPPED", "READY"}:
            if line == "READY":
                return True
        elif line in FAULT_LINES:
            return False
    return False


def send_command_wait_ack(ser, command):
    time.sleep(PRE_SEND_QUIET)
    ser.reset_input_buffer()
    attempt = 0
    while True:
        attempt += 1
        ser.write(command.encode("utf-8"))
        ser.flush()
        deadline = time.time() + ACK_TIMEOUT
        while time.time() < deadline:
            line = read_line_until(ser, deadline)
            if line is None:
                break
            if line in {"AK", "BUSY", "READY"} or line in FAULT_LINES:
                print(f"RX: {line}")
            if line == "AK":
                print("ACK received, waiting for data...")
                return True
            if line == "BUSY":
                print("DSP busy, retry after cooldown")
                time.sleep(COOLDOWN)
                break
            if line in FAULT_LINES:
                raise RuntimeError(f"DSP fault before run: {line}")
            if line == "READY":
                continue
        print(f"No ACK, retry {attempt} until ACK")
        time.sleep(0.5)



def wait_for_ready_after_data(ser):
    deadline = time.time() + POST_DATA_READY_TIMEOUT
    while time.time() < deadline:
        line = read_line_until(ser, deadline)
        if line is None:
            break
        if line == "READY":
            return True
        if line in FAULT_LINES:
            return False
    print("DATA_END received, but READY was not seen")
    return False


def read_data(ser, record_length=RECORD_LENGTH, timeout=DATA_TIMEOUT):
    speed = []
    current = []
    saw_data_end = False
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = read_line_until(ser, deadline)
        if line is None:
            break
        if line in FAULT_LINES:
            raise RuntimeError(f"DSP fault during run: {line}")
        if line == "DATA_START":
            break
    else:
        raise RuntimeError("DATA_START timeout")

    deadline = time.time() + timeout
    while time.time() < deadline:
        line = read_line_until(ser, deadline)
        if line is None:
            break
        if line == "DATA_END":
            saw_data_end = True
            break
        try:
            if len(speed) < record_length:
                parts = [part.strip() for part in line.split(",")]
                speed.append(float(parts[0]))
                if len(parts) >= 4:
                    current.append([float(parts[1]), float(parts[2]), float(parts[3])])
                else:
                    current.append([0.0, 0.0, 0.0])
        except (ValueError, IndexError):
            continue

    if saw_data_end:
        wait_for_ready_after_data(ser)
    if not speed:
        raise RuntimeError("No valid data")
    if len(speed) < record_length:
        speed.extend([speed[-1]] * (record_length - len(speed)))
        current.extend([current[-1]] * (record_length - len(current)))
    return np.asarray(speed[:record_length], dtype=float), np.asarray(current[:record_length], dtype=float)


def command_params(ctrl, params):
    params = list(params)
    fixed_tail = ctrl.get("fixed_tail")
    if fixed_tail is not None and len(params) == len(ctrl["param_names"]):
        return params + list(fixed_tail)
    return params


def build_command(ctrl, mode, params, speed_rpm=None):
    params = command_params(ctrl, params)
    params_txt = ",".join(f"{x:.8g}" for x in params)
    if mode == "bo":
        return f"{ctrl['bo_cmd']},{params_txt}\n"
    if mode == "step":
        return f"{ctrl['high_step_cmd']},{params_txt}\n"
    if mode == "hold":
        return f"{ctrl['hold_cmd']},{speed_rpm:.8g},{params_txt}\n"
    raise ValueError(mode)


def resolve_params(ctrl, params_text):
    if not params_text:
        return list(ctrl["params"])
    params = [float(x.strip()) for x in params_text.split(",") if x.strip()]
    valid_lengths = {len(ctrl["params"]), len(ctrl["param_names"])}
    if len(params) not in valid_lengths:
        raise ValueError(f"{ctrl['name']} needs {sorted(valid_lengths)} params, got {len(params)}")
    return command_params(ctrl, params)


def save_best_params(path, ctrl, params, score):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["controller", ctrl["name"]])
        w.writerow(["score", f"{score:.8g}"])
        full_names = ctrl.get("full_param_names", ctrl["param_names"])
        full_params = command_params(ctrl, params)
        w.writerow(full_names)
        w.writerow([f"{x:.10g}" for x in full_params])


def load_best_params(ctrl_key):
    ctrl = CONTROLLERS[ctrl_key]
    path = BEST_PARAM_DIR / f"{ctrl_key}_best_params.csv"
    if not path.exists():
        return None
    rows = list(csv.reader(path.open("r", encoding="utf-8")))
    if len(rows) < 4:
        return None
    params = [float(x) for x in rows[3] if x.strip()]
    if len(params) != len(ctrl["params"]):
        return None
    return params


def params_for_experiment(ctrl_key, params_text):
    ctrl = CONTROLLERS[ctrl_key]
    if params_text:
        return resolve_params(ctrl, params_text)
    best = load_best_params(ctrl_key)
    if best is not None:
        print(f"使用 {ctrl['label']} 上次 BO 最优参数")
        return best
    print(f"未找到 {ctrl['label']} BO 最优参数，使用默认参数")
    return list(ctrl["params"])


def clear_previous_run(ser):
    # Do not send STOP before every BO trial. STOP closes PWM/DC power in the
    # firmware, causing the drive board to click between parameter sets.
    ser.reset_input_buffer()
    ser.reset_output_buffer()


def cooldown_and_clear(ser, seconds):
    deadline = time.time() + max(0.0, seconds)
    while time.time() < deadline:
        ser.reset_input_buffer()
        time.sleep(min(0.2, max(0.0, deadline - time.time())))


def remove_speed_spikes(speed_array):
    clean = np.asarray(speed_array, dtype=float).copy()
    if len(clean) == 0:
        return clean
    last = clean[0]
    for i in range(1, len(clean)):
        value = clean[i]
        if not np.isfinite(value) or abs(value) > SPIKE_MAX_ABS_RPM or abs(value - last) > SPIKE_MAX_STEP_RPM:
            clean[i] = last
        else:
            last = value
    return clean


def calculate_current_metrics(current_array, ignore_count):
    current_array = np.asarray(current_array, dtype=float)
    if current_array.ndim != 2 or current_array.shape[1] != 3 or len(current_array) == 0:
        return 0.0, 0.0, 0.0
    eval_current = current_array[min(ignore_count, len(current_array) - 1):]
    abs_current = np.abs(eval_current)
    phase_abs = np.max(abs_current, axis=1)
    current_peak = float(np.max(phase_abs))
    current_rms = float(np.sqrt(np.mean(eval_current ** 2)))
    current_penalty = (
        max(0.0, current_peak - CURRENT_WARN_A) * 3000.0
        + max(0.0, current_rms - CURRENT_RMS_WARN_A) * 2000.0
        + max(0.0, current_peak - CURRENT_HARD_A) * 20000.0
    )
    return current_peak, current_rms, current_penalty


def calculate_score(speed_array, current_array=None):
    speed_array = remove_speed_spikes(speed_array)
    speed_abs = np.abs(speed_array)
    ignore_count = min(int(STARTUP_IGNORE_TIME / DT), len(speed_abs) - 1)
    raw_response = speed_abs[ignore_count:]
    if SMOOTHING_WINDOW > 1 and len(speed_abs) >= SMOOTHING_WINDOW:
        kernel = np.ones(SMOOTHING_WINDOW, dtype=float) / SMOOTHING_WINDOW
        pad_left = SMOOTHING_WINDOW // 2
        pad_right = SMOOTHING_WINDOW - 1 - pad_left
        padded = np.pad(speed_abs, (pad_left, pad_right), mode="edge")
        speed_abs = np.convolve(padded, kernel, mode="valid")
    response = speed_abs[ignore_count:]
    start = int(len(response) * (1.0 - TAIL_FRACTION))
    eval_data = response[start:]
    osc_end = max(1, int(len(response) * OSC_EVAL_FRACTION))
    osc_data = response[:osc_end]
    tail_mean = float(np.mean(eval_data))
    tail_max = float(np.max(eval_data))
    overshoot = max(0.0, float(np.max(raw_response) - TARGET_RPM))
    osc_std = float(np.std(osc_data))
    osc_range = float(np.max(osc_data) - np.min(osc_data))
    osc_mean_error = float(np.mean(np.abs(TARGET_RPM - osc_data)))
    osc_penalty = osc_std * OSC_STD_WEIGHT + osc_range * OSC_RANGE_WEIGHT + osc_mean_error * OSC_MEAN_ERROR_WEIGHT
    metrics = {
        "rise_time": DT * len(response),
        "settling_time": DT * len(response),
        "overshoot": overshoot,
        "tail_mean": tail_mean,
        "tail_max": tail_max,
        "osc_std": osc_std,
        "osc_range": osc_range,
        "osc_mean_error": osc_mean_error,
        "osc_penalty": osc_penalty,
        "current_peak": 0.0,
        "current_rms": 0.0,
        "current_penalty": 0.0,
    }
    if current_array is not None:
        metrics["current_peak"], metrics["current_rms"], metrics["current_penalty"] = calculate_current_metrics(current_array, ignore_count)
    if metrics["current_peak"] >= STALL_CURRENT_A and tail_mean < STALL_TAIL_RPM:
        return FAIL_SCORE, metrics
    if tail_mean < MIN_VALID_RPM:
        return FAIL_SCORE, metrics
    rise_indices = np.flatnonzero(response >= TARGET_RPM * RISE_FRACTION)
    if len(rise_indices) > 0:
        metrics["rise_time"] = float(rise_indices[0] * DT)
    band = TARGET_RPM * SETTLING_BAND
    outside = np.flatnonzero(np.abs(response - TARGET_RPM) > band)
    if len(outside) == 0:
        metrics["settling_time"] = 0.0
    elif outside[-1] < len(response) - 1:
        metrics["settling_time"] = float((outside[-1] + 1) * DT)
    t = np.arange(len(eval_data), dtype=float) * DT
    error = np.abs(TARGET_RPM - eval_data)
    steady_itae = float(np.sum(t * error))
    steady_error = abs(TARGET_RPM - tail_mean)
    ripple = float(np.std(eval_data))
    score = (
        metrics["rise_time"] * 4000.0
        + metrics["settling_time"] * 1800.0
        + overshoot * 500.0
        + steady_error * 650.0
        + ripple * 8.0
        + steady_itae * 0.5
        + metrics["osc_penalty"]
        + metrics["current_penalty"]
    )
    return score, metrics


def reference_profile():
    ref = np.empty(STEP_RECORD_LENGTH, dtype=float)
    ref[:STEP1_END_INDEX] = 0.0
    ref[STEP1_END_INDEX:STEP2_END_INDEX] = 100.0
    ref[STEP2_END_INDEX:] = 200.0
    return ref


def save_step_run(out_dir, prefix, speed, current, label):
    out_dir.mkdir(parents=True, exist_ok=True)
    t = np.arange(STEP_RECORD_LENGTH, dtype=float) * DT
    ref = reference_profile()
    csv_path = out_dir / f"{prefix}.csv"
    plot_path = out_dir / f"{prefix}.png"
    np.savetxt(
        csv_path,
        np.column_stack((t, ref, speed, current)),
        delimiter=",",
        header=f"time_s,target_rpm,{label}_speed_rpm,ia_a,ib_a,ic_a",
        comments="",
    )
    fig, (ax_speed, ax_current) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    ax_speed.plot(t, speed, label=f"{label} speed")
    ax_speed.plot(t, ref, "r--", linewidth=1.5, label="target")
    ax_speed.axvline(STEP1_END_INDEX * DT, color="gray", linestyle=":", linewidth=1.0)
    ax_speed.axvline(STEP2_END_INDEX * DT, color="gray", linestyle=":", linewidth=1.0)
    ax_speed.set_ylabel("Speed (rpm)")
    ax_speed.set_title(f"{label} 0-100-200 rpm step response")
    ax_speed.grid(True)
    ax_speed.legend()
    ax_current.plot(t, current[:, 0], label="Ia")
    ax_current.plot(t, current[:, 1], label="Ib")
    ax_current.plot(t, current[:, 2], label="Ic")
    ax_current.axhline(CURRENT_WARN_A, color="orange", linestyle="--", linewidth=1.0, label="current warn")
    ax_current.axhline(-CURRENT_WARN_A, color="orange", linestyle="--", linewidth=1.0)
    ax_current.set_xlabel("Time (s)")
    ax_current.set_ylabel("Current (A)")
    ax_current.grid(True)
    ax_current.legend()
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    return csv_path, plot_path


def save_bo_waveform(out_dir, prefix, speed, current, label):
    out_dir.mkdir(parents=True, exist_ok=True)
    t = np.arange(len(speed), dtype=float) * DT
    csv_path = out_dir / f"{prefix}.csv"
    plot_path = out_dir / f"{prefix}.png"
    np.savetxt(
        csv_path,
        np.column_stack((t, speed, current)),
        delimiter=",",
        header=f"time_s,{label}_speed_rpm,ia_a,ib_a,ic_a",
        comments="",
    )
    fig, (ax_speed, ax_current) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax_speed.plot(t, speed, label=f"{label} speed")
    ax_speed.axhline(TARGET_RPM, color="r", linestyle="--", label="target")
    ax_speed.set_ylabel("Speed (rpm)")
    ax_speed.grid(True)
    ax_speed.legend()
    ax_current.plot(t, current[:, 0], label="Ia")
    ax_current.plot(t, current[:, 1], label="Ib")
    ax_current.plot(t, current[:, 2], label="Ic")
    ax_current.set_xlabel("Time (s)")
    ax_current.set_ylabel("Current (A)")
    ax_current.grid(True)
    ax_current.legend()
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    return csv_path, plot_path


def plot_best_improvements(ax, rounds, plot_best, improved):
    improved = np.asarray(improved, dtype=bool)
    if len(improved) != len(rounds) or not np.any(improved):
        return
    improved_rounds = rounds[improved]
    improved_scores = plot_best[improved]
    ax.plot(improved_rounds, improved_scores, color="goldenrod", linewidth=2.0, label="历史最优")
    last_round = improved_rounds[-1]
    last_score = improved_scores[-1]
    if last_round < rounds[-1]:
        ax.plot([last_round, rounds[-1]], [last_score, last_score], color="goldenrod", linewidth=2.0, label="_nolegend_")
    ax.scatter(improved_rounds, improved_scores, marker="o", s=70, facecolor="yellow", edgecolor="goldenrod", linewidths=1.5, label="刷新最优", zorder=4)


def save_score_plot(log_path, out_dir, improved_history=None):
    trials = []
    scores = []
    if not log_path.exists():
        return None
    with log_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                trials.append(int(float(row["trial"])))
                scores.append(float(row["score"]))
            except Exception:
                pass
    if not trials:
        return None
    rounds = np.asarray(trials, dtype=int)
    plot_scores = np.asarray(scores, dtype=float)
    plot_best = np.minimum.accumulate(plot_scores)
    finite_good = plot_scores[plot_scores < FAIL_SCORE]
    if len(finite_good) > 0:
        cap = max(float(np.max(finite_good)) * 1.15, float(np.min(finite_good)) + 1.0)
        plot_scores = np.minimum(plot_scores, cap)
        plot_best = np.minimum(plot_best, cap)
    if improved_history is None:
        improved_history = np.r_[True, np.diff(plot_best) < 0]
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(rounds, plot_scores, marker="o", linewidth=1.5, label="本轮得分")
    plot_best_improvements(ax, rounds, plot_best, improved_history)
    ax.set_xlabel("??")
    ax.set_ylabel("目标函数得分")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    path = out_dir / "bo_score.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path



def run_once(ser, ctrl_key, mode, params, target_rpm):
    ctrl = CONTROLLERS[ctrl_key]
    clear_previous_run(ser)
    cmd = build_command(ctrl, mode, params, speed_rpm=target_rpm)
    print("Send:", cmd.strip())
    if not send_command_wait_ack(ser, cmd):
        raise RuntimeError("ACK failed")
    if mode == "hold":
        print("Hold command ACK. Motor keeps running until STOP.")
        return None, None
    if mode == "step":
        return read_data(ser, record_length=STEP_RECORD_LENGTH, timeout=STEP_DATA_TIMEOUT)
    return read_data(ser, record_length=RECORD_LENGTH, timeout=DATA_TIMEOUT)


def run_step(args):
    args.controller = normalize_controller(args.controller)
    ctrl = CONTROLLERS[args.controller]
    params = params_for_experiment(args.controller, args.params)
    out = BASE_DIR / ctrl["name"] / "step_0_100_200"
    with serial.Serial(args.port, BAUD_RATE, timeout=0.05) as ser:
        speed, current = run_once(ser, args.controller, "step", params, 200.0)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path, plot_path = save_step_run(out, f"{args.controller}_step_0_100_200_{stamp}", speed, current, args.controller)
    print(f"Saved CSV: {csv_path}")
    print(f"Saved plot: {plot_path}")
    print(
        f"Done. max={np.max(np.abs(speed)):.1f} rpm, "
        f"tail_mean={np.mean(np.abs(speed[STEP2_END_INDEX:])):.1f} rpm, "
        f"Ipk={np.max(np.max(np.abs(current), axis=1)):.2f} A"
    )


def run_hold(args):
    args.controller = normalize_controller(args.controller)
    ctrl = CONTROLLERS[args.controller]
    params = params_for_experiment(args.controller, args.params)
    with serial.Serial(args.port, BAUD_RATE, timeout=0.05) as ser:
        run_once(ser, args.controller, "hold", params, args.target)
    print(f"已进入 {args.target:g} rpm 保持运行，电机会持续转动；需要停止请运行 stop。")


def run_stop(args):
    with serial.Serial(args.port, BAUD_RATE, timeout=0.05) as ser:
        ok = send_stop(ser)
    print("STOP", "ok" if ok else "no response")


def run_bo(args):
    if gp_minimize is None:
        raise RuntimeError("scikit-optimize is required for BO: pip install scikit-optimize")
    args.controller = normalize_controller(args.controller)
    ctrl = CONTROLLERS[args.controller]
    out = BASE_DIR / ctrl["name"] / "bo" / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    log_path = out / "bo_log.csv"
    best = {"score": float("inf"), "params": None, "speed": None, "current": None}
    score_history = []
    best_score_history = []
    improved_history = []
    space = [Real(a, b, name=n) for (a, b), n in zip(ctrl["space"], ctrl["param_names"])]

    with log_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "trial", "status", "score", "rise_time", "settling_time", "overshoot",
            "tail_mean", "tail_max", "osc_std", "osc_range", "osc_mean_error",
            "osc_penalty", "current_peak", "current_rms", "current_penalty",
            *ctrl["param_names"],
        ])

    with serial.Serial(args.port, BAUD_RATE, timeout=0.05) as ser:
        trial = 0
        def objective(params):
            nonlocal trial
            trial += 1
            print(f"\n[Trial {trial}/{args.calls}] " + ", ".join(f"{n}:{v:.5g}" for n, v in zip(ctrl["param_names"], params)))
            status = "ACK_FAIL"
            metrics = {
                "rise_time": 0.0, "settling_time": 0.0, "overshoot": 0.0,
                "tail_mean": 0.0, "tail_max": 0.0, "osc_std": 0.0,
                "osc_range": 0.0, "osc_mean_error": 0.0, "osc_penalty": 0.0,
                "current_peak": 0.0, "current_rms": 0.0, "current_penalty": 0.0,
            }
            score = FAIL_SCORE
            improved = False
            try:
                speed, current = run_once(ser, args.controller, "bo", params, TARGET_RPM)
                score, metrics = calculate_score(speed, current)
                status = "OK" if score < FAIL_SCORE else "LOW_SPEED"
                print(
                    f"Done. score={score:.1f}, rise={metrics['rise_time']:.3f}s, "
                    f"settle={metrics['settling_time']:.3f}s, overshoot={metrics['overshoot']:.1f} rpm, "
                    f"tail_mean={metrics['tail_mean']:.1f} rpm, Ipk={metrics['current_peak']:.2f}A, "
                    f"Irms={metrics['current_rms']:.2f}A"
                )
                if score < best["score"]:
                    best.update(score=score, params=command_params(ctrl, params), speed=speed.copy(), current=current.copy())
                    improved = True
                    save_bo_waveform(out, "best_params_waveform", speed, current, args.controller)
                    save_best_params(out / "best_params.csv", ctrl, params, score)
                    save_best_params(BEST_PARAM_DIR / f"{args.controller}_best_params.csv", ctrl, params, score)
                    print(f"New best score: {score:.1f}")
            except Exception as exc:
                status = "FAIL"
                print("Trial failed:", exc)
            with log_path.open("a", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow([
                    trial, status, score, metrics["rise_time"], metrics["settling_time"],
                    metrics["overshoot"], metrics["tail_mean"], metrics["tail_max"],
                    metrics["osc_std"], metrics["osc_range"], metrics["osc_mean_error"],
                    metrics["osc_penalty"], metrics["current_peak"], metrics["current_rms"],
                    metrics["current_penalty"], *params,
                ])
            score_history.append(score)
            best_score_history.append(min(score_history))
            improved_history.append(improved)
            save_score_plot(log_path, out, improved_history)
            print(f"Cooldown {args.cooldown:.0f}s")
            cooldown_and_clear(ser, args.cooldown)
            return score

        res = gp_minimize(
            objective,
            space,
            n_calls=args.calls,
            n_initial_points=args.initial,
            acq_func="EI",
            random_state=args.seed,
        )
    score_png = save_score_plot(log_path, out, improved_history)
    print("Best score:", best["score"] if best["params"] is not None else res.fun)
    print("Best params:", best["params"] or res.x)
    if score_png:
        print("Score history plot saved to:", score_png)
    print("Best waveform plot saved to:", out / "best_params_waveform.png")



def ask_choice(title, options):
    try:
        import msvcrt
    except Exception:
        msvcrt = None

    if msvcrt is None:
        print(f"\n{title}")
        for i, (key, label) in enumerate(options, start=1):
            print(f"{i}. {label}")
        while True:
            value = input("\u8bf7\u8f93\u5165\u5e8f\u53f7: ").strip()
            if value.isdigit() and 1 <= int(value) <= len(options):
                return options[int(value) - 1][0]
            print("\u8f93\u5165\u65e0\u6548\uff0c\u8bf7\u91cd\u65b0\u9009\u62e9\u3002")

    idx = 0

    def redraw():
        print("\033[2J\033[H", end="")
        print(title)
        print("\u4f7f\u7528 \u2191/\u2193 \u6216 W/S \u9009\u62e9\uff0cEnter \u786e\u8ba4\uff1b\u4e5f\u53ef\u4ee5\u76f4\u63a5\u6309\u6570\u5b57\u3002")
        print()
        for i, (_, label) in enumerate(options):
            prefix = "> " if i == idx else "  "
            print(f"{prefix}{i + 1}. {label}")

    redraw()
    while True:
        ch = msvcrt.getwch()
        if ch in ("\r", "\n"):
            print()
            return options[idx][0]
        if ch.isdigit():
            n = int(ch)
            if 1 <= n <= len(options):
                print()
                return options[n - 1][0]
        if ch in ("w", "W"):
            idx = (idx - 1) % len(options)
            redraw()
        elif ch in ("s", "S"):
            idx = (idx + 1) % len(options)
            redraw()
        elif ch in ("\x00", "\xe0"):
            code = msvcrt.getwch()
            if code == "H":
                idx = (idx - 1) % len(options)
                redraw()
            elif code == "P":
                idx = (idx + 1) % len(options)
                redraw()


def choose_hold_speed():
    return ask_choice(
        "\u8bf7\u9009\u62e9\u4fdd\u6301\u8f6c\u901f",
        [(100.0, "100 rpm \u4fdd\u6301"), (200.0, "200 rpm \u4fdd\u6301")],
    )


def fill_interactive_args(args):
    if args.mode:
        if args.mode == "load":
            args.mode = "hold"
        if args.mode == "bo":
            args.target = 80.0
        elif args.mode == "step":
            args.target = 200.0
        elif args.mode == "hold" and args.target is None:
            args.target = choose_hold_speed()
        return args

    args.controller = ask_choice(
        "\u8bf7\u9009\u62e9\u63a7\u5236\u5668",
        [(key, CONTROLLERS[key]["label"]) for key in ["pi", "ftsmc", "ptsmc"]],
    )
    args.mode = ask_choice(
        "\u8bf7\u9009\u62e9\u8fd0\u884c\u6a21\u5f0f",
        [
            ("bo", "BO \u53c2\u6570\u4f18\u5316\uff080-80 rpm\uff09"),
            ("step", "\u9636\u8dc3\u5b9e\u9a8c\uff080-100-200 rpm\uff09"),
            ("hold", "\u4fdd\u6301\u6307\u5b9a\u8f6c\u901f\u8fd0\u884c"),
            ("stop", "\u505c\u6b62\u7535\u673a"),
        ],
    )
    if args.mode == "bo":
        args.target = 80.0
    elif args.mode == "step":
        args.target = 200.0
    elif args.mode == "hold":
        args.target = choose_hold_speed()
    return args


def main():
    setup_font()
    p = argparse.ArgumentParser(description="单电机轴1/轴2/轴3实验上位机")
    p.add_argument("mode", nargs="?", choices=["bo", "step", "hold", "load", "stop"])
    p.add_argument("--controller", choices=sorted(CONTROLLER_ALIASES), default="ptsmc")
    p.add_argument("--port", default=DSP_PORT)
    p.add_argument("--target", type=float, default=None)
    p.add_argument("--params", help="手动指定参数，逗号分隔；不填则优先使用 BO 最优参数")
    p.add_argument("--calls", type=int, default=40)
    p.add_argument("--initial", type=int, default=15)
    p.add_argument("--cooldown", type=float, default=COOLDOWN)
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()
    args = fill_interactive_args(args)

    if args.controller:
        args.controller = normalize_controller(args.controller)

    if args.mode == "stop":
        run_stop(args)
    elif args.mode == "step":
        run_step(args)
    elif args.mode == "hold":
        run_hold(args)
    elif args.mode == "bo":
        run_bo(args)


if __name__ == "__main__":
    main()
