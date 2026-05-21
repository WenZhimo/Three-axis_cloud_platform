#!/usr/bin/env python3
"""
PID tuning simulator for Three-axis_cloud_platformV2.

Features:
- 3-axis (ROLL/PITCH/YAW) independent PID tuning
- Manual input for 9 PID parameters
- Test modes: step / disturbance / both
- Plot target vs actual angle and control output
- Attempts to load defaults from firmware source files
"""

from __future__ import annotations

import math
import re
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk

from matplotlib import rcParams
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


AXES = ("ROLL", "PITCH", "YAW")
AXIS_LABELS_ZH = {"ROLL": "横滚", "PITCH": "俯仰", "YAW": "航向"}
DT_DEFAULT = 0.002  # 500 Hz
RC_D_FILTER = 1.0 / (2.0 * math.pi * 20.0)  # matches pid.c F_CUT=20Hz

rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Arial Unicode MS"]
rcParams["axes.unicode_minus"] = False


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


@dataclass
class PIDRuntime:
    p: float
    i: float
    d: float
    i_term: float = 0.0
    last_d_calc: float = 0.0
    last_d_term: float = 0.0
    last_last_d_term: float = 0.0
    d_error_calc: bool = True  # D_ERROR


@dataclass
class AxisPlantState:
    theta: float = 0.0
    theta_dot: float = 0.0
    cmd_prev: float = 0.0


@dataclass
class FirmwareDefaults:
    pid: dict[str, dict[str, float]]
    cmd_limit: dict[str, float]
    rate_limit: float


def parse_firmware_defaults(project_root: Path) -> FirmwareDefaults:
    pid_defaults = {
        "ROLL": {"P": 0.005, "I": 0.0, "D": 0.0},
        "PITCH": {"P": 0.1, "I": 0.0, "D": 0.0},
        "YAW": {"P": 10.0, "I": 0.0, "D": 0.016},
    }
    cmd_limit = {"ROLL": 0.30, "PITCH": 0.30, "YAW": 1.00}
    rate_limit = 45.0 * math.pi / 180.0

    main_c = project_root / "Core" / "Src" / "main.c"
    compute_c = project_root / "Drivers" / "SRC" / "Src" / "computeMotorCommands.c"
    config_c = project_root / "Drivers" / "SRC" / "Src" / "config.c"

    if main_c.exists():
        text = main_c.read_text(encoding="utf-8", errors="ignore")
        pattern = re.compile(
            r"eepromConfig\.PID\[(ROLL_PID|PITCH_PID|YAW_PID)\]\.(P|I|D)\s*=\s*([\-+]?\d*\.?\d+)\s*f?"
        )
        for axis_id, term, value in pattern.findall(text):
            axis = axis_id.replace("_PID", "")
            pid_defaults[axis][term] = float(value)

    if compute_c.exists():
        text = compute_c.read_text(encoding="utf-8", errors="ignore")
        limit_pattern = re.compile(
            r"#define\s+(ROLL|PITCH|YAW)_CMD_LIMIT_RAD\s*\(\s*([\-+]?\d*\.?\d+)\s*f?\s*\)"
        )
        for axis, value in limit_pattern.findall(text):
            cmd_limit[axis] = float(value)

    if config_c.exists():
        text = config_c.read_text(encoding="utf-8", errors="ignore")
        deg_pattern = re.search(
            r"eepromConfig\.rateLimit\s*=\s*([\-+]?\d*\.?\d+)\s*f?\s*\*\s*D2R", text
        )
        raw_pattern = re.search(
            r"eepromConfig\.rateLimit\s*=\s*([\-+]?\d*\.?\d+)\s*f?\s*;", text
        )
        if deg_pattern:
            rate_limit = float(deg_pattern.group(1)) * math.pi / 180.0
        elif raw_pattern:
            rate_limit = float(raw_pattern.group(1))

    return FirmwareDefaults(pid=pid_defaults, cmd_limit=cmd_limit, rate_limit=rate_limit)


def update_pid(command: float, state: float, dt: float, i_hold: bool, pid: PIDRuntime) -> float:
    if dt > 0.01 or dt < 0.0001 or not math.isfinite(dt):
        dt = DT_DEFAULT

    error = wrap_pi(command - state)

    if not i_hold:
        pid.i_term += error * dt
        # matches current hard clamp in pid.c
        pid.i_term = clamp(pid.i_term, -10.0, 10.0)

    if pid.last_d_calc == 0.0:
        pid.last_d_calc = state

    if pid.d_error_calc:
        d_term = (error - pid.last_d_calc) / dt
        pid.last_d_calc = error
    else:
        d_term = (pid.last_d_calc - state) / dt
        d_term = wrap_pi(d_term)
        pid.last_d_calc = state

    if not math.isfinite(d_term):
        d_term = 0.0
    d_term = clamp(d_term, -300.0, 300.0)

    d_term_filtered = pid.last_d_term + dt / (RC_D_FILTER + dt) * (d_term - pid.last_d_term)
    if not math.isfinite(d_term_filtered):
        d_term_filtered = 0.0

    d_average = (d_term_filtered + pid.last_d_term + pid.last_last_d_term) / 3.0
    pid.last_last_d_term = pid.last_d_term
    pid.last_d_term = d_term_filtered

    output = pid.p * error + pid.i * pid.i_term + pid.d * d_average
    if not math.isfinite(output):
        output = 0.0
    return output


def run_simulation(
    pid_map: dict[str, PIDRuntime],
    cmd_limit: dict[str, float],
    rate_limit: float,
    sim_time_s: float,
    dt: float,
    step_amp_rad: float,
    step_time_s: float,
    step_axis: str,
    disturbance_mode: str,
    disturbance_amp: float,
    disturbance_time_s: float,
    disturbance_duration_s: float,
    wn: float,
    zeta: float,
    plant_gain: float,
) -> dict[str, list[float] | dict[str, list[float]]]:
    steps = max(1, int(sim_time_s / dt))
    times = [i * dt for i in range(steps)]

    target = {axis: [0.0] * steps for axis in AXES}
    actual = {axis: [0.0] * steps for axis in AXES}
    control = {axis: [0.0] * steps for axis in AXES}

    plant = {axis: AxisPlantState() for axis in AXES}

    mode = disturbance_mode.strip().lower()
    if mode == "阶跃":
        mode = "step"
    elif mode == "扰动":
        mode = "disturbance"
    elif mode == "两者":
        mode = "both"
    use_step = mode in ("step", "both")
    use_disturbance = mode in ("disturbance", "both")
    step_axis = step_axis.strip()
    if step_axis == "横滚":
        step_axis = "ROLL"
    elif step_axis == "俯仰":
        step_axis = "PITCH"
    elif step_axis == "航向":
        step_axis = "YAW"
    elif step_axis == "全部":
        step_axis = "ALL"
    else:
        step_axis = "ROLL"

    for i, t in enumerate(times):
        for axis in AXES:
            apply_step = use_step and t >= step_time_s and (step_axis == "ALL" or axis == step_axis)
            target_now = step_amp_rad if apply_step else 0.0
            target[axis][i] = target_now

            pid = pid_map[axis]
            state = plant[axis]

            u = update_pid(target_now, state.theta, dt, False, pid)
            u = clamp(u, -cmd_limit[axis], cmd_limit[axis])

            du = u - state.cmd_prev
            if du > rate_limit:
                u = state.cmd_prev + rate_limit
            elif du < -rate_limit:
                u = state.cmd_prev - rate_limit
            state.cmd_prev = u
            control[axis][i] = u

            disturbance = 0.0
            if use_disturbance and disturbance_time_s <= t <= disturbance_time_s + disturbance_duration_s:
                disturbance = disturbance_amp

            # Simplified 2nd-order axis model:
            # theta_ddot + 2*zeta*wn*theta_dot + wn^2*theta = plant_gain*u + disturbance
            theta_ddot = (
                plant_gain * u
                + disturbance
                - 2.0 * zeta * wn * state.theta_dot
                - (wn * wn) * state.theta
            )
            state.theta_dot += theta_ddot * dt
            state.theta += state.theta_dot * dt
            state.theta = wrap_pi(state.theta)

            actual[axis][i] = state.theta

    return {
        "time": times,
        "target": target,
        "actual": actual,
        "control": control,
    }


class PIDSimulatorApp:
    def __init__(self, root: tk.Tk, project_root: Path) -> None:
        self.root = root
        self.root.title("三轴 PID 调参仿真器")
        self.project_root = project_root
        self.defaults = parse_firmware_defaults(project_root)

        self.pid_entries: dict[str, dict[str, tk.Entry]] = {}
        self.param_entries: dict[str, tk.Entry] = {}

        self.mode_var = tk.StringVar(value="两者")
        self.step_axis_var = tk.StringVar(value="横滚")
        self._build_ui()
        self._load_defaults_to_ui()
        self.run_and_plot()

    def _build_ui(self) -> None:
        left = ttk.Frame(self.root, padding=8)
        left.pack(side=tk.LEFT, fill=tk.Y)
        right = ttk.Frame(self.root, padding=8)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        ttk.Label(left, text="各轴 PID 参数 (P/I/D)").grid(row=0, column=0, columnspan=4, sticky="w")

        row = 1
        for axis in AXES:
            ttk.Label(left, text=AXIS_LABELS_ZH[axis]).grid(row=row, column=0, sticky="w", padx=(0, 8))
            self.pid_entries[axis] = {}
            for j, term in enumerate(("P", "I", "D")):
                ttk.Label(left, text=term).grid(row=row, column=1 + 2 * j, sticky="e")
                entry = ttk.Entry(left, width=9)
                entry.grid(row=row, column=2 + 2 * j, sticky="w", padx=(2, 8))
                self.pid_entries[axis][term] = entry
            row += 1

        row += 1
        ttk.Label(left, text="模式").grid(row=row, column=0, sticky="w")
        mode_box = ttk.Combobox(
            left,
            textvariable=self.mode_var,
            values=("阶跃", "扰动", "两者"),
            state="readonly",
            width=12,
        )
        mode_box.grid(row=row, column=1, columnspan=2, sticky="w")
        row += 1

        ttk.Label(left, text="阶跃轴").grid(row=row, column=0, sticky="w")
        step_axis_box = ttk.Combobox(
            left,
            textvariable=self.step_axis_var,
            values=("横滚", "俯仰", "航向", "全部"),
            state="readonly",
            width=12,
        )
        step_axis_box.grid(row=row, column=1, columnspan=2, sticky="w")
        row += 1

        params = [
            ("sim_time_s", "仿真时长 (s)", 8.0),
            ("dt_s", "dt (s)", DT_DEFAULT),
            ("step_amp_rad", "阶跃幅值 (rad)", 0.20),
            ("step_time_s", "阶跃时刻 (s)", 0.50),
            ("disturbance_amp", "扰动强度", 5.00),
            ("disturbance_time_s", "扰动时刻 (s)", 2.50),
            ("disturbance_duration_s", "扰动时长 (s)", 0.05),
            ("wn", "对象 wn", 7.0),
            ("zeta", "对象 zeta", 0.85),
            ("plant_gain", "对象增益", 28.0),
            ("rate_limit", "速率限制", self.defaults.rate_limit),
        ]

        for key, label, default_value in params:
            ttk.Label(left, text=label).grid(row=row, column=0, sticky="w")
            entry = ttk.Entry(left, width=12)
            entry.insert(0, f"{default_value:.6g}")
            entry.grid(row=row, column=1, columnspan=2, sticky="w")
            self.param_entries[key] = entry
            row += 1

        btn_frame = ttk.Frame(left)
        btn_frame.grid(row=row, column=0, columnspan=6, sticky="we", pady=(8, 0))
        ttk.Button(btn_frame, text="运行仿真", command=self.run_and_plot).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(btn_frame, text="加载固件默认值", command=self._load_defaults_to_ui).pack(side=tk.LEFT)

        self.figure = Figure(figsize=(10, 6), dpi=100)
        self.ax1 = self.figure.add_subplot(2, 1, 1)
        self.ax2 = self.figure.add_subplot(2, 1, 2)
        self.figure.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.figure, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _load_defaults_to_ui(self) -> None:
        for axis in AXES:
            for term in ("P", "I", "D"):
                entry = self.pid_entries[axis][term]
                entry.delete(0, tk.END)
                entry.insert(0, f"{self.defaults.pid[axis][term]:.8g}")

        if "rate_limit" in self.param_entries:
            self.param_entries["rate_limit"].delete(0, tk.END)
            self.param_entries["rate_limit"].insert(0, f"{self.defaults.rate_limit:.8g}")

    def _float_from_entry(self, key: str) -> float:
        value = self.param_entries[key].get().strip()
        return float(value)

    def run_and_plot(self) -> None:
        try:
            pid_map: dict[str, PIDRuntime] = {}
            cmd_limit = dict(self.defaults.cmd_limit)

            for axis in AXES:
                p = float(self.pid_entries[axis]["P"].get().strip())
                i = float(self.pid_entries[axis]["I"].get().strip())
                d = float(self.pid_entries[axis]["D"].get().strip())
                pid_map[axis] = PIDRuntime(p=p, i=i, d=d)

            sim_time_s = self._float_from_entry("sim_time_s")
            dt = self._float_from_entry("dt_s")
            step_amp_rad = self._float_from_entry("step_amp_rad")
            step_time_s = self._float_from_entry("step_time_s")
            disturbance_amp = self._float_from_entry("disturbance_amp")
            disturbance_time_s = self._float_from_entry("disturbance_time_s")
            disturbance_duration_s = self._float_from_entry("disturbance_duration_s")
            wn = self._float_from_entry("wn")
            zeta = self._float_from_entry("zeta")
            plant_gain = self._float_from_entry("plant_gain")
            rate_limit = self._float_from_entry("rate_limit")

            if sim_time_s <= 0.0 or dt <= 0.0:
                raise ValueError("sim_time_s and dt must be > 0")
            if wn <= 0.0:
                raise ValueError("wn must be > 0")
            if disturbance_duration_s < 0.0:
                raise ValueError("disturbance_duration_s must be >= 0")

            result = run_simulation(
                pid_map=pid_map,
                cmd_limit=cmd_limit,
                rate_limit=rate_limit,
                sim_time_s=sim_time_s,
                dt=dt,
                step_amp_rad=step_amp_rad,
                step_time_s=step_time_s,
                step_axis=self.step_axis_var.get(),
                disturbance_mode=self.mode_var.get(),
                disturbance_amp=disturbance_amp,
                disturbance_time_s=disturbance_time_s,
                disturbance_duration_s=disturbance_duration_s,
                wn=wn,
                zeta=zeta,
                plant_gain=plant_gain,
            )

            t = result["time"]
            target = result["target"]
            actual = result["actual"]
            control = result["control"]

            self.ax1.clear()
            self.ax2.clear()

            colors = {"ROLL": "tab:blue", "PITCH": "tab:orange", "YAW": "tab:green"}
            for axis in AXES:
                c = colors[axis]
                axis_zh = AXIS_LABELS_ZH[axis]
                self.ax1.plot(t, target[axis], "--", color=c, alpha=0.50, label=f"{axis_zh} 目标")
                self.ax1.plot(t, actual[axis], "-", color=c, linewidth=1.5, label=f"{axis_zh} 实际")
                self.ax2.plot(t, control[axis], "-", color=c, linewidth=1.5, label=f"{axis_zh} 控制输出")

            self.ax1.set_title("目标角度 与 实际角度")
            self.ax1.set_ylabel("角度 (rad)")
            self.ax1.grid(True, alpha=0.3)
            self.ax1.legend(loc="upper right", ncol=2, fontsize=8)

            self.ax2.set_title("控制输出（PID 指令）")
            self.ax2.set_xlabel("时间 (s)")
            self.ax2.set_ylabel("指令 (rad)")
            self.ax2.grid(True, alpha=0.3)
            self.ax2.legend(loc="upper right", ncol=3, fontsize=8)

            self.figure.tight_layout(pad=2.0)
            self.canvas.draw_idle()

        except Exception as exc:
            messagebox.showerror("输入错误", str(exc))


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    root = tk.Tk()
    PIDSimulatorApp(root, project_root=project_root)
    root.mainloop()


if __name__ == "__main__":
    main()
