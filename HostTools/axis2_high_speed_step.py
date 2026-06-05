import time
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import serial


DSP_PORT = "COM3"
BAUD_RATE = 115200
RECORD_LENGTH = 4000
DT = 0.001

READY_TIMEOUT = 8.0
ACK_TIMEOUT = 8.0
DATA_TIMEOUT = 35.0
PRE_SEND_QUIET = 2.0

BEST_PI_AXIS2 = [
    9.89938537,
    0.00592039,
]

BASE_DIR = Path(__file__).with_name("axis2_highspeed_result")
BASE_DIR.mkdir(exist_ok=True)


def setup_chinese_font():
    for font_path in (
        Path(r"C:\Windows\Fonts\msyh.ttc"),
        Path(r"C:\Windows\Fonts\simhei.ttf"),
        Path(r"C:\Windows\Fonts\simsun.ttc"),
    ):
        if font_path.exists():
            font_manager.fontManager.addfont(str(font_path))
            font_name = font_manager.FontProperties(fname=str(font_path)).get_name()
            plt.rcParams["font.family"] = "sans-serif"
            plt.rcParams["font.sans-serif"] = [font_name, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False


setup_chinese_font()


try:
    ser = serial.Serial(DSP_PORT, BAUD_RATE, timeout=0.05)
    print(f"Connected to DSP serial port: {DSP_PORT}")
except Exception as exc:
    print(f"Failed to open serial port: {exc}")
    raise SystemExit(1)


def read_line_until(deadline):
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="ignore").strip()
        if line:
            return line
    return None


def wait_for_ready_or_quiet():
    deadline = time.time() + READY_TIMEOUT
    saw_line = False
    while time.time() < deadline:
        line = read_line_until(deadline)
        if line is None:
            break
        saw_line = True
        if line == "READY":
            return True
        if line == "BUSY":
            return False
        if line in {"RUN_FAIL", "NO_DC", "HALL_FAIL", "STALL", "DC_DROP"}:
            print(f"   DSP residual status: {line}")
            continue
        if line in {"DATA_START", "DATA_END", "DUAL_START", "DUAL_END"}:
            continue
    return not saw_line


def wait_for_ack(cmd):
    time.sleep(PRE_SEND_QUIET)
    attempt = 0
    while True:
        attempt += 1
        if not wait_for_ready_or_quiet():
            print(f"   DSP not ready, retry {attempt} until ACK")
            time.sleep(0.5)
            continue

        ser.write(cmd.encode("utf-8"))
        ser.flush()

        deadline = time.time() + ACK_TIMEOUT
        while time.time() < deadline:
            line = read_line_until(deadline)
            if line is None:
                break
            if line == "AK":
                return True
            if line == "BUSY":
                print("   DSP busy, retrying")
                break
            if line in {"RUN_FAIL", "NO_DC", "HALL_FAIL", "STALL", "DC_DROP"}:
                print(f"   DSP fault before run: {line}")
                return False
            if line == "READY":
                continue

        print(f"   No ACK, retry {attempt} until ACK")
        time.sleep(0.5)


def receive_speed_data():
    speed = []
    ia = []
    ib = []
    ic = []
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

    deadline = time.time() + DATA_TIMEOUT
    while time.time() < deadline:
        line = read_line_until(deadline)
        if line is None:
            break
        if line == "DATA_END":
            break
        try:
            if len(speed) < RECORD_LENGTH:
                parts = [float(part) for part in line.split(",")]
                speed.append(parts[0])
                ia.append(parts[1] if len(parts) > 1 else 0.0)
                ib.append(parts[2] if len(parts) > 2 else 0.0)
                ic.append(parts[3] if len(parts) > 3 else 0.0)
        except ValueError:
            continue

    deadline = time.time() + 3.0
    while time.time() < deadline:
        line = read_line_until(deadline)
        if line is None:
            break
        if line == "READY":
            break

    if not speed:
        return None

    if len(speed) < RECORD_LENGTH:
        missing = RECORD_LENGTH - len(speed)
        speed.extend([speed[-1]] * missing)
        ia.extend([ia[-1] if ia else 0.0] * missing)
        ib.extend([ib[-1] if ib else 0.0] * missing)
        ic.extend([ic[-1] if ic else 0.0] * missing)

    return (
        np.asarray(speed[:RECORD_LENGTH], dtype=float),
        np.asarray(ia[:RECORD_LENGTH], dtype=float),
        np.asarray(ib[:RECORD_LENGTH], dtype=float),
        np.asarray(ic[:RECORD_LENGTH], dtype=float),
    )


def remove_spikes(speed, max_abs=700.0, max_step=300.0):
    clean = speed.copy()
    last = clean[0]
    changed = 0
    for i in range(1, len(clean)):
        value = clean[i]
        if not np.isfinite(value) or abs(value) > max_abs or abs(value - last) > max_step:
            clean[i] = last
            changed += 1
        else:
            last = value
    return clean, changed


def save_result(trace):
    speed, ia, ib, ic = trace
    t = np.arange(len(speed), dtype=float) * DT
    target = np.zeros_like(t)
    target[t >= 1.0] = 100.0
    target[t >= 2.5] = 200.0
    clean, spike_count = remove_spikes(speed)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = BASE_DIR / f"axis2_highstep_0_100_200_{timestamp}.csv"
    png_path = BASE_DIR / f"axis2_highstep_0_100_200_{timestamp}.png"

    np.savetxt(
        csv_path,
        np.column_stack((t, target, speed, clean, ia, ib, ic)),
        delimiter=",",
        header="time_s,target_rpm,speed_axis2_pi_raw_rpm,speed_axis2_pi_filtered_rpm,ia_axis2_a,ib_axis2_a,ic_axis2_a",
        comments="",
    )

    fig, (ax, ax_i) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax.step(t, target, where="post", color="r", linestyle="--", linewidth=1.2, label="目标转速")
    ax.plot(t, clean, linewidth=1.8, label="轴2滤波后实际转速")
    if spike_count:
        ax.plot(t, speed, color="0.75", linewidth=0.8, alpha=0.7, label="原始速度")
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("转速 (rpm)")
    ax.set_title("轴2固定最优PI参数高速多阶跃实验")
    ax.grid(True, alpha=0.6)
    ax.legend(loc="upper left")
    ax_i.plot(t, ia, linewidth=1.0, label="Ia")
    ax_i.plot(t, ib, linewidth=1.0, label="Ib")
    ax_i.plot(t, ic, linewidth=1.0, label="Ic")
    ax_i.set_xlabel("时间 (s)")
    ax_i.set_ylabel("电流 (A)")
    ax_i.grid(True, alpha=0.6)
    ax_i.legend(loc="upper left", ncol=3)
    param_text = (
        "固定最优参数\n"
        f"Kp={BEST_PI_AXIS2[0]:.8f}\n"
        f"Ki={BEST_PI_AXIS2[1]:.8f}\n"
        "step: 0-1.0s 0rpm, 1.0-2.5s 100rpm, 2.5-4.0s 200rpm"
    )
    ax.text(
        0.98,
        0.03,
        param_text,
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        fontsize=8,
        bbox=dict(facecolor="white", edgecolor="0.4", alpha=0.82),
    )
    fig.tight_layout()
    fig.savefig(png_path, dpi=180)
    plt.close(fig)

    print(f"   Saved CSV: {csv_path}")
    print(f"   Saved plot: {png_path}")
    print(f"   raw min/max={np.min(speed):.2f}/{np.max(speed):.2f}")
    print(f"   filtered min/max={np.min(clean):.2f}/{np.max(clean):.2f}, tail_mean={np.mean(clean[-200:]):.2f}")
    print(f"   filtered spike points: {spike_count}")


def main():
    cmd = "HSTEP2," + ",".join(f"{value:.8f}" for value in BEST_PI_AXIS2) + "\n"

    print("=====================================================")
    print("Starting axis-2 fixed-parameter high-speed multi-step experiment")
    print(f"Axis 2 PI Kp={BEST_PI_AXIS2[0]:.8f}, Ki={BEST_PI_AXIS2[1]:.8f}")
    print("=====================================================")

    print("\n[Axis 2 High-speed Run: 0 rpm 1.0s, 100 rpm 1.5s, 200 rpm 1.5s]")
    if not wait_for_ack(cmd):
        print("   ACK failed")
    else:
        print("   ACK received, waiting for data...")
        trace = receive_speed_data()
        if trace is None:
            print("   No valid speed data")
        else:
            save_result(trace)

    print("\nAxis-2 high-speed experiment complete")
    print(f"Results folder: {BASE_DIR}")


if __name__ == "__main__":
    main()
