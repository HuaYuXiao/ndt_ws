"""
Figure 1: Compliant contact mechanism — spring-damper passive compliance on UAV end-effector.
Illustrates how kinetic impact energy is absorbed, and why spring restoring force couples F_n to deflection.
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

fig, axes = plt.subplots(1, 3, figsize=(12, 4.2), facecolor="white")

# Panel (a): Spring-damper end-effector schematic
ax = axes[0]
ax.set_xlim(0, 6); ax.set_ylim(0, 6); ax.axis("off")
# UAV body (simplified)
body = mpatches.Rectangle((2.0, 4.5), 2.0, 1.2, fc="#5B8DB8", ec="#2A4A6A", lw=1.5, alpha=0.85)
ax.add_patch(body)
ax.text(3.0, 5.1, "UAV frame", ha="center", fontsize=8, color="#2A4A6A", fontweight="bold")
# Spring
spring_y = [3.3, 3.6, 3.0, 3.6, 3.0, 3.6, 3.0, 3.6, 3.3]
spring_x = np.linspace(2.6, 3.4, len(spring_y))
ax.plot(spring_x, spring_y, color="#D4660A", lw=1.8, solid_capstyle="round")
ax.text(3.2, 3.85, "k", fontsize=9, color="#D4660A", fontstyle="italic")
# Damper
damper_y = [3.3, 2.9]
ax.plot([3.0, 3.0], damper_y, color="#888", lw=2)
ax.plot([2.7, 3.3], [3.1, 3.1], color="#888", lw=2)
ax.plot([2.7, 3.3], [2.9, 2.9], color="#888", lw=2)
ax.text(2.3, 3.0, "b", fontsize=9, color="#888", fontstyle="italic")
# Probe
probe = mpatches.Rectangle((2.4, 1.8), 1.2, 1.0, fc="#E07020", ec="#8B4513", lw=1.2, alpha=0.8)
ax.add_patch(probe)
ax.text(3.0, 2.3, "Probe", ha="center", fontsize=7, color="white", fontweight="bold")
# Surface
ax.plot([0.5, 5.5], [1.2, 1.2], color="#3A5F80", lw=2.5)
ax.fill_between([0.5, 5.5], 0, 1.2, fc="#7BA0C4", alpha=0.4)
ax.text(3.0, 0.9, "Test surface", ha="center", fontsize=8, color="#3A5F80")
# Contact force arrow
ax.annotate(r"$F_n = k \cdot \Delta x$", xy=(3.0, 2.6), xytext=(4.3, 3.5),
            fontsize=9, color="#D4660A", ha="center",
            arrowprops=dict(arrowstyle="->", color="#D4660A", lw=1.5))
ax.set_title("(a) Passive compliant mechanism", fontsize=10, fontweight="bold", pad=6)

# Panel (b): Force-displacement coupling problem
ax = axes[1]
ax.set_xlim(0, 5); ax.set_ylim(0, 5)
# Two stiffness curves
x = np.linspace(0, 4, 100)
ax.plot(x, 0.5 + 2.0 * x, color="#D4660A", lw=2, label=r"High stiffness $k_H$")
ax.plot(x, 0.5 + 0.6 * x, color="#2980B9", lw=2, label=r"Low stiffness $k_L$")
# EMAT stable zone
ax.fill_between([0, 3.5], 0, 1.0, alpha=0.15, color="#27AE60")
ax.text(2.5, 0.3, "EMAT stable zone (< 1 N fluctuation)", fontsize=7, color="#27AE60", ha="center")
# Position error band
ax.axvspan(0.8, 1.2, alpha=0.15, color="#E74C3C")
ax.text(1.0, 4.5, r"$\Delta x$ error", fontsize=8, color="#E74C3C", ha="center")
# Force variation annotations
ax.annotate("Force variation\nwith $k_H$", xy=(1.0, 2.5), xytext=(2.5, 3.8),
            fontsize=7, color="#D4660A", arrowprops=dict(arrowstyle="->", color="#D4660A", lw=1))
ax.annotate("Force variation\nwith $k_L$", xy=(1.0, 1.1), xytext=(2.8, 1.8),
            fontsize=7, color="#2980B9", arrowprops=dict(arrowstyle="->", color="#2980B9", lw=1))
ax.set_xlabel("Probe deflection (mm)", fontsize=8)
ax.set_ylabel("Contact force (N)", fontsize=8)
ax.legend(fontsize=7, loc="lower right")
ax.set_title("(b) Force uncontrolled by deflection", fontsize=10, fontweight="bold", pad=6)

# Panel (c): Impact dynamics
ax = axes[2]
t = np.linspace(0, 2, 200)
fn = 0.2 + 3 * np.exp(-6 * t) * np.sin(12 * np.pi * t) * (t < 0.7) + 0.5 * np.exp(-3 * (t - 0.7)) * (t > 0.7)
ax.plot(t, fn, color="#E07020", lw=1.8)
ax.axvspan(0, 0.15, alpha=0.12, color="#E74C3C")
ax.text(0.08, 3.3, "Impact", fontsize=7, color="#E74C3C", ha="center")
ax.axvspan(0.15, 0.7, alpha=0.12, color="#F39C12")
ax.text(0.45, 3.3, "Oscillation", fontsize=7, color="#F39C12", ha="center")
ax.axvspan(0.7, 2.0, alpha=0.12, color="#27AE60")
ax.text(1.35, 3.3, "Steady (uncontrolled)", fontsize=7, color="#27AE60", ha="center")
ax.set_xlabel("Time (s)", fontsize=8)
ax.set_ylabel(r"Contact force $F_n$ (N)", fontsize=8)
ax.set_title("(c) Impact transient response", fontsize=10, fontweight="bold", pad=6)

plt.tight_layout(pad=2)
plt.savefig("figures/fig_contact_passive_compliance.pdf", bbox_inches="tight", dpi=300)
plt.savefig("figures/fig_contact_passive_compliance.png", bbox_inches="tight", dpi=300)
plt.close()
print("fig_contact_passive_compliance saved")
