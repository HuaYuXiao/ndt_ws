"""
Figure 3: UAV contact inspection platform evolution — from passive to over-actuated.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "Arial"],
    "font.size": 8.5,
    "axes.spines.right": False, "axes.spines.top": False,
    "axes.linewidth": 0.6,
})

fig, ax = plt.subplots(figsize=(12, 7), facecolor="white")
ax.set_xlim(0, 14); ax.set_ylim(0, 7.5); ax.axis("off")

def draw_platform(ax, cx, cy, rotors, tilt_angle, arms_angle, body_color, label, has_arm=False, arm_angle=0):
    """Draw a simplified multirotor with given geometry."""
    cos_a, sin_a = arms_angle[0], arms_angle[1]  # cosine and sine of arm angles
    # Body
    body = mpatches.Ellipse((cx, cy), 1.0, 0.35, angle=np.degrees(np.arctan2(tilt_angle[1], tilt_angle[0])),
                             fc=body_color, ec="#333", lw=0.8, alpha=0.85)
    ax.add_patch(body)
    # Arms and rotors
    colors = ["#888", "#888", "#888", "#888"]
    for i, (rm_angle_cos, rm_angle_sin) in enumerate(zip(
        [cos_a[i] for i in range(4)], [sin_a[i] for i in range(4)])):
        rx = cx + 1.6 * cos_a[i]
        ry = cy + 1.6 * sin_a[i]
        ax.plot([cx + 0.4 * cos_a[i], cx + 1.2 * cos_a[i]],
                [cy + 0.12 * sin_a[i], cy + 1.2 * sin_a[i]],
                color="#555", lw=1.2)
        circle = plt.Circle((rx, ry), 0.28, fc="#95A5A6", ec="#555", lw=0.8, alpha=0.7)
        ax.add_patch(circle)
    # End-effector
    probe_x = cx + 0.8 * tilt_angle[1]
    probe_y = cy - 0.6 + 0.4 * tilt_angle[0]
    ax.plot([cx, probe_x], [cy - 0.2, probe_y], color="#E07020", lw=2.5, solid_capstyle="round")
    ax.plot(probe_x, probe_y, "o", color="#E07020", ms=5)
    # Label
    ax.text(cx, cy - 1.2, label, ha="center", fontsize=8, fontweight="bold", color="#333")

# ── (a) Underactuated platform: tilt-contact conflict ──
cx_a, cy_a = 2.5, 5.8
rotor_positions = [(0.866, 0.5), (-0.866, 0.5), (-0.866, -0.5), (0.866, -0.5)]
tilt = (0.2588, 0.9659)  # ~15 deg tilt
arms = ([0.707, -0.707, -0.707, 0.707], [0.707, 0.707, -0.707, -0.707])
draw_platform(ax, cx_a, cy_a, 4, tilt, arms, "#5B8DB8", "Under-actuated")

# Surface (vertical wall)
ax.plot([4.8, 4.8], [4.0, 6.8], color="#3A5F80", lw=3)
ax.fill_between([4.8, 6.0], 4.0, 6.8, fc="#7BA0C4", alpha=0.3)
ax.text(5.4, 5.4, "Wall", fontsize=9, color="#3A5F80", fontweight="bold")

# Tilt angle arc
from matplotlib.patches import Arc
arc = Arc((cx_a, cy_a + 0.3), 1.5, 0.8, angle=0, theta1=10, theta2=55, color="#D4660A", lw=1.5)
ax.add_patch(arc)
ax.text(cx_a + 1.5, cy_a + 1.2, r"$\theta_{tilt}$", fontsize=9, color="#D4660A")

# Conflict annotation
ax.annotate(r"$\theta_{tilt} \Rightarrow$ probe $\not\perp$ surface",
            xy=(3.5, 5.0), xytext=(1.5, 3.5),
            fontsize=8, color="#C0392B", fontweight="bold",
            arrowprops=dict(arrowstyle="->", color="#C0392B", lw=1.2),
            bbox=dict(boxstyle="round,pad=0.2", fc="#FDEDEC", alpha=0.8))

# Arrow from UAV to wall
ax.annotate("", xy=(4.7, 5.8), xytext=(3.5, 5.6),
            arrowprops=dict(arrowstyle="->", color="#D4660A", lw=1.5))
ax.text(4.2, 5.3, r"$F_n$", fontsize=9, color="#D4660A", fontweight="bold")

# ── (b) Over-actuated platform: decoupled ──
cx_b, cy_b = 9.5, 5.8
tilt_b = (0.0175, 0.9998)  # ~1 deg tilt
draw_platform(ax, cx_b, cy_b, 4, tilt_b, arms, "#2980B9", "Over-actuated")

# Surface (vertical wall)
ax.plot([12.0, 12.0], [4.0, 6.8], color="#3A5F80", lw=3)
ax.fill_between([12.0, 13.2], 4.0, 6.8, fc="#7BA0C4", alpha=0.3)
ax.text(12.6, 5.4, "Wall", fontsize=9, color="#3A5F80", fontweight="bold")

# Decoupled annotation
ax.annotate(r"$\theta_{tilt} \approx 0$, probe $\perp$ surface",
            xy=(cx_b, 4.8), xytext=(7.5, 3.5),
            fontsize=8, color="#27AE60", fontweight="bold",
            arrowprops=dict(arrowstyle="->", color="#27AE60", lw=1.2),
            bbox=dict(boxstyle="round,pad=0.2", fc="#E8F8F5", alpha=0.8))

# Arrow from UAV to wall
ax.annotate("", xy=(12.3, 5.8), xytext=(10.5, 5.7),
            arrowprops=dict(arrowstyle="->", color="#2980B9", lw=1.5))
ax.text(11.5, 5.4, r"$F_n$", fontsize=9, color="#2980B9", fontweight="bold")

# ── (c) Platform evolution timeline ──
line_y = 1.8
ax.plot([0.8, 13.2], [line_y, line_y], color="#555", lw=1.5)
# Markers
for x, label, color in [(2.0, "Passive\nCompliance\n(2019-)", "#D4660A"),
                          (6.0, "Active Force\nControl (2019-)", "#C0392B"),
                          (9.0, "Over-actuated\nPlatform (2020-)", "#2980B9"),
                          (12.0, "Perception-\nControl Fusion\n(this work)", "#27AE60")]:
    ax.plot(x, line_y, "o", color=color, ms=10, mec="#333", mew=0.8)
    ax.text(x, line_y - 0.6, label, ha="center", fontsize=7.5, color=color, fontweight="bold")

# Title
ax.text(7.0, 7.2, "UAV Contact Inspection Platform Evolution",
        ha="center", fontsize=13, fontweight="bold", color="#1A2A3A")

# Legend at bottom
legend_text = [
    ("Under-actuated: tilt-force coupling", "#5B8DB8"),
    ("Over-actuated: position-attitude decoupled", "#2980B9"),
    ("Mechanical coupling path", "#D4660A"),
    ("Decoupled coupling path", "#2980B9"),
]
for i, (txt, color) in enumerate(legend_text):
    ax.text(2.5 + i * 3.5, 0.6, txt, fontsize=7.5, color=color, ha="center",
            fontweight="bold")

plt.tight_layout(pad=1.5)
plt.savefig("figures/fig_uav_platform_evolution.pdf", bbox_inches="tight", dpi=300)
plt.savefig("figures/fig_uav_platform_evolution.png", bbox_inches="tight", dpi=300)
plt.close()
print("fig_uav_platform_evolution saved")
