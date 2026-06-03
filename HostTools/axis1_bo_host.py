import time
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import serial
from skopt import gp_minimize
from skopt.space import Real


def setup_chinese_font():
    font_candidates = [
        Path(r"C:\Windows\Fonts\msyh.ttc"),
        Path(r"C:\Windows\Fonts\simhei.ttf"),
        Path(r"C:\Windows\Fonts\simsun.ttc"),
    ]
    for font_path in font_candidates:
        if font_path.exists():
            font_manager.fontManager.addfont(str(font_path))
            font_name = font_manager.FontProperties(fname=str(font_path)).get_name()
            plt.rcParams["font.sans-serif"] = [font_name, "DejaVu Sans"]
            plt.rcParams["font.family"] = "sans-serif"
            plt.rcParams["axes.unicode_minus"] = False
            return font_name
    plt.rcParams["axes.unicode_minus"] = False
    return None


CHINESE_FONT_NAME = setup_chinese_font()

DSP_PORT = "COM3"
BAUD_RATE = 115200
TARGET_RPM = 80.0
RECORD_LENGTH = 2000
DT = 0.001
COOLDOWN = 10.0
N_CALLS = 40
N_INITIAL_POINTS = 15

FAIL_SCORE = 200000.0
READY_TIMEOUT = 2.0
ACK_TIMEOUT = 6.0
DATA_TIMEOUT = 15.0
POST_DATA_READY_TIMEOUT = 3.0
MAX_ACK_ATTEMPTS = 6
PRE_SEND_QUIET = 1.5

# Do not score the forced-start transient. The DSP now always returns data unless
# there is a hardware fault, so low-speed/no-motion trials are rejected here.
MIN_VALID_RPM = TARGET_RPM * 0.75
TAIL_FRACTION = 0.50
STARTUP_IGNORE_TIME = 0.10
RISE_FRACTION = 0.90
SETTLING_BAND = 0.08
SMOOTHING_WINDOW = 25
SPIKE_MAX_ABS_RPM = 300.0
SPIKE_MAX_STEP_RPM = 150.0
RUN_TIMESTAMP = time.strftime("%Y%m%d_%H%M%S")
RESULT_DIR = Path(__file__).with_name("axis1_bo_results")
RESULT_DIR.mkdir(exist_ok=True)
LOG_PATH = RESULT_DIR / f"axis1_bo_results_{RUN_TIMESTAMP}.csv"
SCORE_PLOT_PATH = RESULT_DIR / f"axis1_bo_score_history_{RUN_TIMESTAMP}.png"
BEST_SPEED_CSV_PATH = RESULT_DIR / f"axis1_best_speed_2000pts_{RUN_TIMESTAMP}.csv"
BEST_SPEED_PLOT_PATH = RESULT_DIR / f"axis1_best_speed_waveform_{RUN_TIMESTAMP}.png"
LIVE_SCORE_PLOT = True


try:
    ser = serial.Serial(DSP_PORT, BAUD_RATE, timeout=0.05)
    print(f"Connected to DSP serial port: {DSP_PORT}")
except Exception as exc:
    print(f"Failed to open serial port: {exc}")
    raise SystemExit(1)


best_speed_data = None
best_params = None
best_score = float("inf")
score_history = []
best_score_history = []
best_improved_history = []
score_fig = None
score_ax = None

PARAM_NAMES = ("P1", "P2", "A11", "B11", "A22", "B22", "C3", "P3", "A33", "B33", "C8")

LOG_PATH.write_text(
    "trial,status,score,rise_time,settling_time,overshoot,tail_mean,tail_max,"
    + ",".join(PARAM_NAMES)
    + "\n",
    encoding="utf-8",
)
trial_count = 0


def append_result(status, score, metrics, params):
    with LOG_PATH.open("a", encoding="utf-8") as fp:
        param_text = ",".join(f"{value:.8f}" for value in params)
        fp.write(
            f"{trial_count},{status},{score:.6f},"
            f"{metrics['rise_time']:.6f},{metrics['settling_time']:.6f},"
            f"{metrics['overshoot']:.6f},{metrics['tail_mean']:.6f},"
            f"{metrics['tail_max']:.6f},"
            f"{param_text}\n"
        )


def format_best_params_text():
    if best_params is None:
        return "最佳参数：无"

    lines = [f"最佳得分：{best_score:.2f}", "最佳参数："]
    for i in range(0, len(PARAM_NAMES), 4):
        chunk = zip(PARAM_NAMES[i:i + 4], best_params[i:i + 4])
        lines.append("  " + ", ".join(f"{name}={value:.5g}" for name, value in chunk))
    return "\n".join(lines)


def plot_best_improvements(ax, rounds, plot_best, improved):
    if len(improved) != len(rounds) or not np.any(improved):
        return

    improved_rounds = rounds[improved]
    improved_scores = plot_best[improved]
    ax.plot(
        improved_rounds,
        improved_scores,
        color="goldenrod",
        linewidth=2.0,
        label="历史最低得分",
    )
    last_round = improved_rounds[-1]
    last_score = improved_scores[-1]
    if last_round < rounds[-1]:
        ax.plot(
            [last_round, rounds[-1]],
            [last_score, last_score],
            color="goldenrod",
            linewidth=2.0,
            label="_nolegend_",
        )
    ax.scatter(
        improved_rounds,
        improved_scores,
        marker="o",
        s=70,
        facecolor="yellow",
        edgecolor="goldenrod",
        linewidths=1.5,
        label="刷新最低分",
        zorder=4,
    )


def update_score_plot():
    global score_fig, score_ax

    if not LIVE_SCORE_PLOT or not score_history:
        return

    rounds = np.arange(1, len(score_history) + 1)
    plot_scores = np.asarray(score_history, dtype=float)
    plot_best = np.asarray(best_score_history, dtype=float)
    improved = np.asarray(best_improved_history, dtype=bool)

    finite_good = plot_scores[plot_scores < FAIL_SCORE]
    if len(finite_good) > 0:
        cap = max(float(np.max(finite_good)) * 1.15, float(np.min(finite_good)) + 1.0)
        plot_scores = np.minimum(plot_scores, cap)
        plot_best = np.minimum(plot_best, cap)

    if score_fig is None or score_ax is None:
        plt.ion()
        score_fig, score_ax = plt.subplots(figsize=(10, 5))

    score_ax.clear()
    score_ax.plot(rounds, plot_scores, marker="o", linewidth=1.5, label="本轮得分")
    plot_best_improvements(score_ax, rounds, plot_best, improved)
    score_ax.set_xlabel("轮次")
    score_ax.set_ylabel("目标函数得分")
    score_ax.set_title("轴1贝叶斯优化得分记录")
    score_ax.grid(True)
    score_ax.legend()
    score_fig.tight_layout()
    score_fig.savefig(SCORE_PLOT_PATH, dpi=150)
    plt.pause(0.01)



def read_line_until(deadline):
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="ignore").strip()
        if line:
            return line
    return None


def wait_for_ack(cmd):
    """Send one command, then only resend if DSP explicitly reports BUSY/timeout."""
    time.sleep(PRE_SEND_QUIET)
    attempt = 0
    while True:
        attempt += 1
        if attempt == 1:
            ser.reset_input_buffer()
        ser.write(cmd.encode("utf-8"))
        ser.flush()

        deadline = time.time() + ACK_TIMEOUT
        while time.time() < deadline:
            line = read_line_until(deadline)
            if line is None:
                break
            if line == "AK":
                return True
            if line in {"RUN_FAIL", "NO_DC", "HALL_FAIL", "STALL", "DC_DROP"}:
                print(f"   DSP fault before run: {line}")
                return False
            if line == "BUSY":
                print("   DSP busy, retry after cooldown")
                time.sleep(COOLDOWN)
                break
            if line == "READY":
                continue

        print(f"   No ACK, retry {attempt} until ACK")
        time.sleep(0.5)


def wait_for_ready_after_data():
    deadline = time.time() + POST_DATA_READY_TIMEOUT
    while time.time() < deadline:
        line = read_line_until(deadline)
        if line is None:
            break
        if line == "READY":
            return True
        if line in {"RUN_FAIL", "NO_DC", "HALL_FAIL", "STALL", "DC_DROP"}:
            print(f"   DSP fault after data: {line}")
            return False

    print("   DATA_END received, but READY was not seen")
    return False


def receive_speed_data():
    speed_data = []
    saw_data_end = False
    deadline = time.time() + DATA_TIMEOUT

    while time.time() < deadline:
        line = read_line_until(deadline)
        if line is None:
            break
        if line == "DATA_START":
            break
        if line in {"RUN_FAIL", "NO_DC", "HALL_FAIL", "STALL", "DC_DROP"}:
            print(f"   DSP fault: {line}")
            return None
    else:
        return None

    deadline = time.time() + DATA_TIMEOUT
    while time.time() < deadline:
        line = read_line_until(deadline)
        if line is None:
            break
        if line == "DATA_END":
            saw_data_end = True
            break
        try:
            if len(speed_data) < RECORD_LENGTH:
                speed_data.append(float(line))
        except ValueError:
            continue

    if saw_data_end:
        wait_for_ready_after_data()

    if not speed_data:
        return None

    if len(speed_data) < RECORD_LENGTH:
        speed_data.extend([speed_data[-1]] * (RECORD_LENGTH - len(speed_data)))

    return np.asarray(speed_data[:RECORD_LENGTH], dtype=float)


def remove_speed_spikes(speed_array):
    clean = np.asarray(speed_array, dtype=float).copy()
    if len(clean) == 0:
        return clean

    last = clean[0]
    for i in range(1, len(clean)):
        value = clean[i]
        if (
            not np.isfinite(value)
            or abs(value) > SPIKE_MAX_ABS_RPM
            or abs(value - last) > SPIKE_MAX_STEP_RPM
        ):
            clean[i] = last
        else:
            last = value
    return clean


def calculate_score(speed_array):
    speed_array = remove_speed_spikes(speed_array)
    speed_abs = np.abs(speed_array)
    if SMOOTHING_WINDOW > 1 and len(speed_abs) >= SMOOTHING_WINDOW:
        kernel = np.ones(SMOOTHING_WINDOW, dtype=float) / SMOOTHING_WINDOW
        pad_left = SMOOTHING_WINDOW // 2
        pad_right = SMOOTHING_WINDOW - 1 - pad_left
        padded = np.pad(speed_abs, (pad_left, pad_right), mode="edge")
        speed_abs = np.convolve(padded, kernel, mode="valid")

    ignore_count = min(int(STARTUP_IGNORE_TIME / DT), len(speed_abs) - 1)
    response = speed_abs[ignore_count:]
    start = int(len(response) * (1.0 - TAIL_FRACTION))
    eval_data = response[start:]

    tail_mean = float(np.mean(eval_data))
    tail_max = float(np.max(eval_data))
    overshoot = max(0.0, float(np.max(response) - TARGET_RPM))

    metrics = {
        "rise_time": DT * len(response),
        "settling_time": DT * len(response),
        "overshoot": overshoot,
        "tail_mean": tail_mean,
        "tail_max": tail_max,
    }

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

    return (
        metrics["rise_time"] * 4000.0
        + metrics["settling_time"] * 1800.0
        + overshoot * 100.0
        + steady_error * 50.0
        + ripple * 8.0
        + steady_itae * 0.5
    ), metrics


def run_dsp_experiment(params):
    global best_speed_data, best_params, best_score, trial_count
    trial_count += 1

    param_map = dict(zip(PARAM_NAMES, params))
    param_text = ", ".join(f"{name}:{param_map[name]:.4g}" for name in PARAM_NAMES)
    print(f"\n[Trial {trial_count}/{N_CALLS}] {param_text}")

    cmd = "P," + ",".join(f"{value:.6g}" for value in params) + "\n"
    score = FAIL_SCORE
    status = "ACK_FAIL"
    metrics = {
        "rise_time": 0.0,
        "settling_time": 0.0,
        "overshoot": 0.0,
        "tail_mean": 0.0,
        "tail_max": 0.0,
    }

    improved_best = False

    if wait_for_ack(cmd):
        print("   ACK received, waiting for data...")
        speed_data = receive_speed_data()
        if speed_data is None:
            print("   No valid speed data")
            status = "NO_DATA"
        else:
            score, metrics = calculate_score(speed_data)
            tail = np.abs(speed_data[int(RECORD_LENGTH * (1.0 - TAIL_FRACTION)):])
            status = "OK" if score < FAIL_SCORE else "LOW_SPEED"
            print(
                f"   Done. score={score:.1f}, "
                f"rise={metrics['rise_time']:.3f}s, "
                f"settle={metrics['settling_time']:.3f}s, "
                f"overshoot={metrics['overshoot']:.1f} rpm, "
                f"tail_mean={metrics['tail_mean']:.1f} rpm"
            )
            if score < best_score:
                best_score = score
                best_params = tuple(params)
                best_speed_data = speed_data.copy()
                improved_best = True
                print(f"   New best score: {best_score:.1f}")
    else:
        print("   ACK failed")

    append_result(status, score, metrics, params)
    score_history.append(score)
    best_score_history.append(min(score_history))
    best_improved_history.append(improved_best)
    update_score_plot()
    print(f"   Cooldown {COOLDOWN:.0f}s")
    time.sleep(COOLDOWN)
    return score


# TC1, TC2 and TC3 are fixed in the DSP. These 11 bounds follow the
# PTSMC+PTDO parameter structure in the paper, but are kept as HIL search
# ranges for this motor instead of copying one paper motor's final values.
space = [
    Real(0.85, 0.99, name="P1"),
    Real(0.50, 0.95, name="P2"),
    Real(20.0, 80.0, name="A11"),
    Real(0.05, 5.0, name="B11"),
    Real(20.0, 90.0, name="A22"),
    Real(220.0, 320.0, name="B22"),
    Real(0.80, 1.20, name="C3"),
    Real(0.85, 0.99, name="P3"),
    Real(100.0, 180.0, name="A33"),
    Real(20.0, 90.0, name="B33"),
    Real(50.0, 500.0, name="C8"),
]

print("=====================================================")
print("Starting HIL Bayesian optimization")
print("=====================================================")

res = gp_minimize(
    run_dsp_experiment,
    space,
    n_calls=N_CALLS,
    n_initial_points=N_INITIAL_POINTS,
    acq_func="EI",
    random_state=42,
)

print("\nOptimization complete")
print(f"Best score: {res.fun:.2f}")
print(f"Best params: {res.x}")

if score_history:
    rounds = np.arange(1, len(score_history) + 1)
    plot_scores = np.asarray(score_history, dtype=float)
    plot_best = np.asarray(best_score_history, dtype=float)
    improved = np.asarray(best_improved_history, dtype=bool)

    # Keep failed/low-speed trials visible without flattening the useful score range.
    finite_good = plot_scores[plot_scores < FAIL_SCORE]
    if len(finite_good) > 0:
        cap = max(float(np.max(finite_good)) * 1.15, float(np.min(finite_good)) + 1.0)
        plot_scores = np.minimum(plot_scores, cap)
        plot_best = np.minimum(plot_best, cap)

    plt.figure(figsize=(10, 5))
    ax = plt.gca()
    ax.plot(rounds, plot_scores, marker="o", linewidth=1.5, label="本轮得分")
    plot_best_improvements(ax, rounds, plot_best, improved)
    plt.xlabel("轮次")
    plt.ylabel("目标函数得分")
    plt.title("轴1贝叶斯优化得分记录")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(SCORE_PLOT_PATH, dpi=150)
    print(f"Score history plot saved to: {SCORE_PLOT_PATH}")

if best_speed_data is not None:
    t = np.arange(len(best_speed_data)) * DT
    np.savetxt(
        BEST_SPEED_CSV_PATH,
        np.column_stack((t, best_speed_data)),
        delimiter=",",
        header="time_s,best_axis1_speed_rpm",
        comments="",
    )
    plt.figure(figsize=(10, 5))
    plt.plot(t, best_speed_data, label="轴1转速")
    plt.axhline(y=TARGET_RPM, color="r", linestyle="--", linewidth=2, label="目标转速")
    plt.xlabel("时间 (s)")
    plt.ylabel("转速 (rpm)")
    plt.title("轴1最佳参数控制下的转速响应")
    plt.gca().text(
        0.98,
        0.02,
        format_best_params_text(),
        transform=plt.gca().transAxes,
        va="bottom",
        ha="right",
        fontsize=8,
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "0.55", "alpha": 0.88},
    )
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(BEST_SPEED_PLOT_PATH, dpi=150)
    print(f"Best speed data saved to: {BEST_SPEED_CSV_PATH}")
    print(f"Best speed plot saved to: {BEST_SPEED_PLOT_PATH}")
    plt.show()
