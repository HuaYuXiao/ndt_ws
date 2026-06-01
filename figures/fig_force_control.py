"""
Figure 2: Active force control with NMPC and force-signal coupling block diagram.
"""
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "Arial"],
    "font.size": 9,
    "axes.spines.right": False, "axes.spines.top": False,
    "axes.linewidth": 0.6,
})

fig, axes = plt.subplots(1, 2, figsize=(12, 5), facecolor="white")

# Panel (a): Active force control block diagram
ax = axes[0]
ax.set_xlim(0, 12); ax.set_ylim(0, 8); ax.axis("off")

def draw_block(ax, x, y, w, h, text, color, text_color="white"):
    rect = mpatches.FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1",
                                    fc=color, ec="#333", lw=0.8, alpha=0.9)
    ax.add_patch(rect)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
            fontsize=7, color=text_color, fontweight="bold")

def draw_arrow(ax, x1, y1, x2, y2, color="#555"):
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="->", color=color, lw=1.2))

# Blocks
draw_block(ax, 1.0, 5.5, 2.2, 1.0, "NMPC\nTrajectory\nOptimizer", "#2980B9")
draw_block(ax, 4.0, 5.5, 2.0, 1.0, "UAV\nFlight\nController", "#5B8DB8")
draw_block(ax, 6.8, 5.5, 1.8, 1.0, "Multi-rotor\nDynamics", "#7BA0C4")
draw_block(ax, 4.0, 3.0, 2.0, 1.0, "Contact Force\nSensor", "#8E44AD")
draw_block(ax, 1.0, 3.0, 2.2, 1.0, "NMHE\nForce\nEstimator", "#9B59B6")
draw_block(ax, 9.5, 5.5, 1.8, 1.0, "EMAT\nSignal\nQuality", "#E07020")

# Arrows - control loop
draw_arrow(ax, 3.2, 6.0, 4.0, 6.0); draw_arrow(ax, 6.0, 6.0, 6.8, 6.0)
draw_arrow(ax, 8.6, 6.0, 9.5, 6.0)
# Feedback
draw_arrow(ax, 11.3, 5.5, 11.3, 2.0, "#D35400")
draw_arrow(ax, 11.3, 2.0, 5.0, 3.5, "#D35400")
draw_arrow(ax, 6.0, 3.5, 6.0, 4.0)
draw_arrow(ax, 5.0, 3.5, 3.2, 3.5)
draw_arrow(ax, 3.2, 3.5, 2.1, 4.0)

# Labels
ax.text(0.5, 6.0, r"$F_n^{ref}$", fontsize=8, color="#2980B9", va="center", fontweight="bold")
ax.text(11.5, 5.0, "Echo", fontsize=7, color="#E07020", ha="center")
ax.text(11.5, 3.0, r"$h, \theta_n$", fontsize=7, color="#555", ha="center")
ax.text(7.5, 3.0, r"$F_n^{meas}$", fontsize=7, color="#8E44AD", ha="center")
ax.text(0.5, 3.1, r"$\hat{F}_n^{ext}$", fontsize=7, color="#9B59B6", va="center")

# Force reference + estimated force -> NMPC
ax.annotate("+", xy=(2.1, 4.2), fontsize=11, color="#333", ha="center")
ax.plot([0.8, 2.1], [5.5, 4.2], "k-", lw=0.8)
ax.plot([2.1, 2.1], [4.1, 4.0], "k-", lw=0.8)

ax.set_title("(a) Active force control with NMPC/NMHE — signal flow", fontsize=10, fontweight="bold", pad=6)

# Panel (b): Force-signal coupling — EMAT specific
ax = axes[1]
h_vals = np.linspace(0.3, 5, 100)

# Two y-axes showing F_n (force) vs B (flux ≈ signal amplitude)
ax2 = ax.twinx()

# Force for two stiffness values
Fn_kH = 0.5 + 3.5 * h_vals  # high stiffness - force goes up linearly with compression
Fn_kL = 0.5 + 0.8 * h_vals  # low stiffness

# Flux (signal amplitude) — nonlinear decay
B = 1.0 * h_vals**(-1.8)

ax.plot(h_vals, Fn_kH, color="#D4660A", lw=2, ls="--", label=r"$F_n$ (high $k$)")
ax.plot(h_vals, Fn_kL, color="#D4660A", lw=1.2, ls=":", label=r"$F_n$ (low $k$)")
ax2.plot(h_vals, B, color="#2980B9", lw=2.5, label=r"$\|\mathbf{B}\|$ (flux)")
ax2.plot(h_vals, B * (1 + 0.1 * np.sin(h_vals * 8)), color="#2980B9", lw=0.8, alpha=0.3)

# Working zone
ax2.fill_between([0.5, 2.0], 0, 0.8, alpha=0.1, color="#27AE60")
ax.text(1.25, 7.5, "EMAT stable\nworking zone", fontsize=7, color="#27AE60", ha="center")

ax.annotate(r"$F_n$ stable $\neq$ $\|\mathbf{B}\|$ stable",
            xy=(1.5, 1.7), xytext=(3.5, 2.5),
            fontsize=8, color="#8E44AD", fontweight="bold",
            arrowprops=dict(arrowstyle="->", color="#8E44AD", lw=1.2))

ax.set_xlabel("Lift-off distance (mm)", fontsize=8)
ax.set_ylabel("Contact force (N)", fontsize=8, color="#D4660A")
ax2.set_ylabel(r"Magnetic flux $\|\mathbf{B}\|$ (a.u.)", fontsize=8, color="#2980B9")
ax.tick_params(axis="y", colors="#D4660A")
ax2.tick_params(axis="y", colors="#2980B9")
ax.set_title("(b) Force-coupling decoupling: $F_n$ vs. EM flux", fontsize=10, fontweight="bold", pad=6)

# Legend
lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax.legend(lines1 + lines2, labels1 + labels2, fontsize=7, loc="upper right")

plt.tight_layout(pad=2)
plt.savefig("figures/fig_force_control_nmpc.pdf", bbox_inches="tight", dpi=300)
plt.savefig("figures/fig_force_control_nmpc.png", bbox_inches="tight", dpi=300)
plt.close()
print("fig_force_control_nmpc saved")
