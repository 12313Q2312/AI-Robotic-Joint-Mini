from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "processed"
OUT = ROOT / "assets" / "plots"
OUT.mkdir(parents=True, exist_ok=True)


def read_csv(name: str) -> list[dict[str, float]]:
    with (DATA / name).open("r", encoding="utf-8-sig", newline="") as handle:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(handle)
        ]


def values(rows: list[dict[str, float]], key: str) -> list[float]:
    return [row[key] for row in rows]


def degrees(rows: list[dict[str, float]], key: str) -> list[float]:
    return [math.degrees(row[key]) for row in rows]


p = read_csv("experiment_0002_p_control.csv")
pd = read_csv("experiment_0003_pd_control.csv")
pd_load = read_csv("experiment_0004_pd_constant_load.csv")
pid_load = read_csv("experiment_0005_pid_constant_load.csv")
anti = read_csv("experiment_0006_anti_windup.csv")

target_deg = math.degrees(p[0]["target_rad"])

plt.style.use("seaborn-v0_8-whitegrid")
fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)

ax = axes[0]
ax.plot(values(p, "time_s"), degrees(p, "actual_rad"), linewidth=2.2, label="P")
ax.plot(values(pd, "time_s"), degrees(pd, "actual_rad"), linewidth=2.2, label="PD")
ax.axhline(target_deg, color="black", linestyle="--", linewidth=1.4, label="Target 30 deg")
ax.set_title("No external load: P vs PD")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Joint angle (deg)")
ax.legend()

ax = axes[1]
ax.plot(values(pd_load, "time_s"), degrees(pd_load, "actual_rad"), linewidth=2.2, label="PD + load")
ax.plot(values(pid_load, "time_s"), degrees(pid_load, "actual_rad"), linewidth=2.2, label="PID + load")
ax.axhline(target_deg, color="black", linestyle="--", linewidth=1.4, label="Target 30 deg")
ax.set_title("Constant -0.02 N m load: PD vs PID")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Joint angle (deg)")
ax.legend()

fig.suptitle("Experiment 001: Joint position response", fontsize=14, fontweight="bold")
fig.savefig(OUT / "experiment_001_response_comparison.png", dpi=200)
plt.close(fig)

fig, ax = plt.subplots(figsize=(9, 5), constrained_layout=True)
time = values(pid_load, "time_s")
ax.plot(time, values(pid_load, "p_torque_nm"), linewidth=2, label="P torque")
ax.plot(time, values(pid_load, "i_torque_nm"), linewidth=2, label="I torque")
ax.plot(time, values(pid_load, "d_torque_nm"), linewidth=2, label="D torque")
ax.plot(time, values(pid_load, "control_torque_nm"), linewidth=2.3, label="Controller total")
ax.plot(time, values(pid_load, "load_torque_nm"), linestyle="--", linewidth=2, label="External load")
ax.axhline(0.0, color="black", linewidth=1)
ax.set_title("PID torque components under constant load")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Torque (N m)")
ax.legend(ncol=2)
fig.savefig(OUT / "experiment_001_pid_torque_components.png", dpi=200)
plt.close(fig)

fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True, constrained_layout=True)
time = values(anti, "time_s")
raw = values(anti, "raw_control_torque_nm")
limited = values(anti, "control_torque_nm")
integral = values(anti, "error_integral_rad_s")
active = values(anti, "anti_windup_active")

ax = axes[0]
ax.plot(time, raw, linewidth=2.2, label="Raw controller torque")
ax.plot(time, limited, linewidth=2.2, label="Limited torque")
ax.axhline(0.04, color="red", linestyle="--", linewidth=1.5, label="Temporary +0.04 N m limit")
ax.set_ylabel("Torque (N m)")
ax.set_title("Anti-windup validation")
ax.legend()

ax = axes[1]
ax.plot(time, integral, color="#6a3d9a", linewidth=2.2, label="Integral state")
ax.fill_between(time, 0, max(integral) if integral else 1.0, where=[flag > 0.5 for flag in active], color="#fb9a99", alpha=0.45, step="mid", label="Anti-windup active")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Integral state (rad s)")
ax.legend()
fig.savefig(OUT / "experiment_001_anti_windup_validation.png", dpi=200)
plt.close(fig)

print(OUT / "experiment_001_response_comparison.png")
print(OUT / "experiment_001_pid_torque_components.png")
print(OUT / "experiment_001_anti_windup_validation.png")
