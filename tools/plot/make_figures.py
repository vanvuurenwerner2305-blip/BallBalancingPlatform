"""Generate the figures of docs/physics from the simulator's verification output.

    build/Release/bbp_tests --csv build/csv
    build/Release/bbp_sim_demo build/demo
    python tools/plot/make_figures.py build/csv build/demo docs/physics/figures
"""
import csv
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

CSV, DEMO, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)

# Reference categorical palette, slots 1-3 in fixed order; identity is also carried by
# line style / markers so the figures survive greyscale printing.
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED, GRID, AXIS = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"

plt.rcParams.update({
    "figure.figsize": (3.4, 2.3), "figure.dpi": 150, "savefig.bbox": "tight",
    "font.family": "serif", "font.size": 8, "axes.labelsize": 8, "legend.fontsize": 7,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.5,
    "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.4,
    "legend.frameon": False,
})


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def save(fig, name):
    fig.savefig(os.path.join(OUT, name))
    plt.close(fig)


def analytic(ax, x, y, label="analytic"):
    ax.plot(x, y, color=INK, lw=0.9, ls=(0, (4, 3)), label=label)


# 1. Rolling down an incline: solid vs hollow sphere
fig, ax = plt.subplots()
for name, col, mk, lab in [("solid", BLUE, "o", r"solid, $k=2/5$"), ("hollow", ORANGE, "s", r"shell, $k=0.604$")]:
    d = load(os.path.join(CSV, f"ball_incline_{name}.csv"))
    ax.plot(d["t"], d["s"] * 1e3, color=col, marker=mk, markevery=600, ms=4, label=lab)
    analytic(ax, d["t"], d["s_analytic"] * 1e3, label=None)
ax.plot([], [], color=INK, lw=0.9, ls=(0, (4, 3)), label=r"$\frac{1}{2}\frac{g\sin\beta}{1+k}t^2$")
ax.set_xlabel("time [s]")
ax.set_ylabel("distance rolled [mm]")
ax.legend()
save(fig, "incline.pdf")

# 2. Sliding to rolling
d = load(os.path.join(CSV, "ball_slide.csv"))
fig, ax = plt.subplots()
ax.plot(d["t"] * 1e3, d["v"], color=BLUE, label=r"centre speed $v$")
ax.plot(d["t"] * 1e3, d["a_omega"], color=ORANGE, ls="--", label=r"$a\,\omega$")
k, mu, g, v0 = 0.4, 0.4, 9.80665, 0.3
ts = k * v0 / ((1 + k) * mu * g)
ax.axvline(ts * 1e3, color=MUTED, lw=0.7)
ax.annotate(r"$t^* = \frac{k v_0}{(1+k)\mu_k g}$", (ts * 1e3, 0.12), xytext=(ts * 1e3 + 8, 0.08), color=INK2,
            arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.6))
ax.axhline(v0 / (1 + k), color=INK, lw=0.9, ls=(0, (4, 3)))
ax.text(70, v0 / (1 + k) + 0.008, r"$v_0/(1+k)$", color=INK2)
ax.set_xlim(0, 100)
ax.set_xlabel("time [ms]")
ax.set_ylabel("speed [m/s]")
ax.legend(loc="lower right")
save(fig, "slide.pdf")

# 3. Turntable orbit
d = load(os.path.join(CSV, "ball_turntable.csv"))
fig, ax = plt.subplots(figsize=(2.6, 2.6))
ax.plot(d["x"] * 1e3, d["y"] * 1e3, color=BLUE)
ax.plot(d["x"][0] * 1e3, d["y"][0] * 1e3, "o", color=BLUE, ms=5)
ax.plot(0, 0, "+", color=INK, ms=8)
ax.text(2, 3, "turntable axis", color=INK2)
ax.set_aspect("equal")
ax.set_xlabel("x [mm]")
ax.set_ylabel("y [mm]")
save(fig, "turntable.pdf")

# 4. Rolling resistance
d = load(os.path.join(CSV, "ball_rolling_resistance.csv"))
fig, ax = plt.subplots()
ax.plot(d["t"], d["v"], color=BLUE, label="simulated")
dec = 0.01 * 9.80665 / 1.4
analytic(ax, d["t"], np.maximum(0.2 - dec * d["t"], 0), label=r"$v_0 - \frac{c_{rr} g}{1+k} t$")
ax.set_xlabel("time [s]")
ax.set_ylabel("speed [m/s]")
ax.legend()
save(fig, "rolling_resistance.pdf")

# 5. Parasitic motion and crank angles
d = load(os.path.join(CSV, "kin_parasitic.csv"))
fig, axs = plt.subplots(1, 2, figsize=(6.8, 2.3))
sel = d["azimuth_deg"] == 0
axs[0].plot(d["tilt_deg"][sel], d["parasitic_mm"][sel], color=BLUE, marker="o", markevery=5, ms=4,
            label="from kinematics")
analytic(axs[0], d["tilt_deg"][sel], d["closed_form_mm"][sel], label=r"$\frac{r_B}{2}(1-\cos\theta)$")
axs[0].set_xlabel("tilt [deg]")
axs[0].set_ylabel("parasitic shift [mm]")
axs[0].legend()
sel = d["tilt_deg"] == 10
for i, (col, ls) in enumerate([(BLUE, "-"), (ORANGE, "--"), (AQUA, ":")]):
    axs[1].plot(d["azimuth_deg"][sel], d[f"theta{i}_deg"][sel], color=col, ls=ls, marker="os^"[i], ms=3,
                label=f"crank {i}")
axs[1].set_xlabel("tilt azimuth [deg] (tilt = 10 deg)")
axs[1].set_ylabel("crank angle [deg]")
axs[1].legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.18))
save(fig, "kinematics.pdf")

# 6. Refraction through the plate
d = load(os.path.join(CSV, "cam_slab_shift.csv"))
fig, ax = plt.subplots()
ax.plot(d["incidence_deg"], d["shift_mm"], color=BLUE, marker="o", ms=4, ls="none", label="ray tracer")
analytic(ax, d["incidence_deg"], d["analytic_mm"], label=r"$t\sin(i-r)/\cos r$")
ax.set_xlabel("angle of incidence [deg]")
ax.set_ylabel("lateral ray shift [mm]")
ax.legend()
save(fig, "slab_shift.pdf")

# 7. Servo step response
d = load(os.path.join(CSV, "servo_step.csv"))
fig, ax = plt.subplots()
ax.plot(d["t"] * 1e3, d["theta_deg"], color=BLUE, label="crank angle")
ax.step(d["t"] * 1e3, d["target_deg"], where="post", color=INK, lw=0.9, ls=(0, (4, 3)), label="servo target")
ax.set_xlim(0, 200)
ax.set_xlabel("time [ms]")
ax.set_ylabel("angle [deg]")
ax.legend(loc="lower right")
save(fig, "servo_step.pdf")

# 8. Closed loop (camera in the loop)
d = load(os.path.join(DEMO, "closed_loop.csv"))
fig, axs = plt.subplots(2, 1, figsize=(6.8, 3.6), sharex=True)
for ax, c in zip(axs, "xy"):
    ax.plot(d["t"], d[f"true_{c}"] * 1e3, color=BLUE, label="true position")
    ax.plot(d["t"], d[f"meas_{c}"] * 1e3, color=ORANGE, ls="none", marker=".", ms=2, label="camera measurement")
    ax.step(d["t"], d[f"target_{c}"] * 1e3, where="post", color=INK, lw=0.9, ls=(0, (4, 3)), label="setpoint")
    ax.set_ylabel(f"{c} [mm]")
axs[0].legend(ncol=3, loc="upper center", bbox_to_anchor=(0.5, 1.3))
axs[1].set_xlabel("time [s]")
save(fig, "closed_loop.pdf")


# 9. Rendered frames (PNG)
def ppm(p):
    data = open(p, "rb").read()
    parts = data.split(b"\n", 3)
    w, h = map(int, parts[1].split())
    return np.frombuffer(parts[3], dtype=np.uint8).reshape(h, w, 3)


for src, dst in [(os.path.join(DEMO, "frame_135.ppm"), "frame_noisy.png"),
                 (os.path.join(CSV, "cam_frame.ppm"), "frame_clean.png")]:
    plt.imsave(os.path.join(OUT, dst), ppm(src))

# 10. Silhouette centroid bias
d = load(os.path.join(CSV, "cam_centroid.csv"))
fig, ax = plt.subplots()
ax.plot(d["radius_mm"], d["err_mm"], color=BLUE, marker="o", ms=4)
ax.set_xlabel("ball distance from plate centre [mm]")
ax.set_ylabel("centroid error [mm]")
save(fig, "centroid_bias.pdf")
print("figures written to", OUT)
