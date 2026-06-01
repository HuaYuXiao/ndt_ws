"""
Nature-style EMAT electromagnetic-elastic coupling schematic.
Isometric 3D illustration with Lorentz force, eddy currents, and formula cards.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch
from matplotlib.collections import LineCollection
import matplotlib.lines as mlines

# ── Nature journal style ──
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["DejaVu Serif", "Times New Roman", "serif"],
    "font.size": 10,
    "axes.linewidth": 0.6,
    "figure.dpi": 150,
    "svg.fonttype": "none",
    "pdf.fonttype": 42,
})

# ── Colors ──
BG = "#F4F6F8"
METAL_TOP = "#A8C4E0"
METAL_SIDE1 = "#7BA0C4"
METAL_SIDE2 = "#6089AE"
METAL_EDGE = "#3A5F80"
COIL_COLOR = "#D4660A"
MAGNET_RED = "#C0392B"
MAGNET_BLUE = "#2980B9"
EDDY_COLOR = "#E8852A"
FORCE_ORANGE = "#E07020"
FORCE_YELLOW = "#F0A030"
FIELD_COLOR = "#7090B0"
TEXT_COLOR = "#1A2A3A"
FORMULA_BG = "#E8ECF0"
FORMULA_EDGE = "#B0BEC5"
ANNOT_COLOR = "#455A64"
DASHED = "#78909C"
HIGHLIGHT = "#FF8F00"

fig, ax = plt.subplots(figsize=(16, 9), facecolor=BG)
ax.set_facecolor(BG)
ax.set_xlim(0, 16)
ax.set_ylim(0, 9)
ax.set_aspect("equal")
ax.axis("off")
fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01)


def iso(x, y, z):
    """Isometric projection: 3D -> 2D."""
    sx = (x - y) * np.cos(np.radians(30))
    sy = (x + y) * np.sin(np.radians(30)) + z
    return sx, sy


def iso_line(x1, y1, z1, x2, y2, z2, **kwargs):
    sx1, sy1 = iso(x1, y1, z1)
    sx2, sy2 = iso(x2, y2, z2)
    ax.plot([sx1, sx2], [sy1, sy2], **kwargs)


def iso_polygon(coords_3d, **kwargs):
    pts = [iso(*c) for c in coords_3d]
    xs = [p[0] for p in pts] + [pts[0][0]]
    ys = [p[1] for p in pts] + [pts[0][1]]
    ax.fill(xs, ys, **kwargs)
    return pts


def iso_text(x, y, z, text, **kwargs):
    sx, sy = iso(x, y, z)
    ax.text(sx, sy, text, **kwargs)


# ── Offset to center the scene ──
OX, OY = 7.5, 3.5


def p(x, y, z):
    sx, sy = iso(x, y, z)
    return sx + OX, sy + OY


def p_line(x1, y1, z1, x2, y2, z2, **kwargs):
    x1s, y1s = p(x1, y1, z1)
    x2s, y2s = p(x2, y2, z2)
    ax.plot([x1s, x2s], [y1s, y2s], **kwargs)


def p_polygon(coords_3d, **kwargs):
    pts = [p(*c) for c in coords_3d]
    xs = [pt[0] for pt in pts] + [pts[0][0]]
    ys = [pt[1] for pt in pts] + [pts[0][1]]
    ax.fill(xs, ys, **kwargs)
    return pts


def p_text(x, y, z, text, **kwargs):
    sx, sy = p(x, y, z)
    ax.text(sx, sy, text, **kwargs)


def p_arrow(x, y, z, dx, dy, dz, **kwargs):
    x1, y1 = p(x, y, z)
    x2, y2 = p(x + dx, y + dy, z + dz)
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="->", **kwargs))


# ══════════════════════════════════════════════════════════════
# 1. METAL BLOCK
# ══════════════════════════════════════════════════════════════
bx, by, bz = 3.5, 3.5, 1.5

# Right face
p_polygon([(0, 0, 0), (0, by, 0), (0, by, bz), (0, 0, bz)],
          color=METAL_SIDE2, alpha=0.85, zorder=2)
# Front face
p_polygon([(0, 0, 0), (bx, 0, 0), (bx, 0, bz), (0, 0, bz)],
          color=METAL_SIDE1, alpha=0.85, zorder=2)
# Top face
p_polygon([(0, 0, bz), (bx, 0, bz), (bx, by, bz), (0, by, bz)],
          color=METAL_TOP, alpha=0.85, zorder=3)

# Edges
for x0, y0, z0, x1, y1, z1 in [
    (0, 0, 0, bx, 0, 0), (0, 0, 0, 0, by, 0), (0, 0, 0, 0, 0, bz),
    (bx, 0, 0, bx, 0, bz), (bx, 0, 0, bx, by, 0),
    (0, by, 0, 0, by, bz), (0, by, 0, bx, by, 0),
    (bx, by, 0, bx, by, bz),
    (0, 0, bz, bx, 0, bz), (0, 0, bz, 0, by, bz),
    (bx, 0, bz, bx, by, bz), (0, by, bz, bx, by, bz),
]:
    p_line(x0, y0, z0, x1, y1, z1, color=METAL_EDGE, lw=0.8, zorder=4)

# Metal label
pxc, pyc = p(bx * 0.5, by * 0.5, bz * 0.5)
ax.text(pxc, pyc, "Conductive\nMaterial", ha="center", va="center",
        fontsize=9, color="white", fontweight="bold", zorder=5,
        fontfamily="serif")

# Surface label
pxs, pys = p(bx * 0.5, by * 0.5, bz + 0.15)
ax.text(pxs, pys, "Surface", ha="center", va="bottom",
        fontsize=8, color=METAL_EDGE, fontstyle="italic", zorder=5)

# ══════════════════════════════════════════════════════════════
# 2. COIL (spiral above metal)
# ══════════════════════════════════════════════════════════════
cx, cy, cz_base = 1.2, 1.2, bz + 1.8
coil_R = 0.7
n_turns = 4
coil_height = 1.2

for i in range(n_turns):
    t = np.linspace(0, 2 * np.pi, 60)
    phase = i * 2 * np.pi / n_turns * 0.15
    z_offset = i * coil_height / n_turns
    r_coil = coil_R * (1 + 0.05 * np.sin(t * 3))

    xs_3d = cx + r_coil * np.cos(t + phase)
    ys_3d = cy + r_coil * np.sin(t + phase)
    zs_3d = cz_base + z_offset + 0.08 * np.sin(t * 2)

    pts = [p(x, y, z) for x, y, z in zip(xs_3d, ys_3d, zs_3d)]
    sx = [pt[0] for pt in pts]
    sy = [pt[1] for pt in pts]

    # Color gradient: bottom warmer, top cooler
    frac = i / max(n_turns - 1, 1)
    col = plt.cm.YlOrBr(0.3 + 0.5 * frac)

    ax.plot(sx, sy, color=col, lw=2.0, solid_capstyle="round", zorder=12)

# Coil center axis (thin dashed)
p_line(cx, cy, cz_base - 0.3, cx, cy, cz_base + coil_height + 0.5,
       color=COIL_COLOR, lw=0.5, ls="--", alpha=0.4, zorder=10)

# Current direction arrow (on top turn)
arrow_t = np.linspace(0.3, 1.2, 30)
arrow_r = coil_R * np.ones_like(arrow_t)
arrow_x = cx + arrow_r * np.cos(arrow_t)
arrow_y = cy + arrow_r * np.sin(arrow_t)
arrow_z = cz_base + coil_height + np.zeros_like(arrow_t)
pts_arr = [p(x, y, z) for x, y, z in zip(arrow_x, arrow_y, arrow_z)]
ax.annotate("", xy=pts_arr[-1], xytext=pts_arr[-3],
            arrowprops=dict(arrowstyle="-|>", color=COIL_COLOR,
                            lw=1.8, mutation_scale=12), zorder=13)

# Current label
pt_curl = p(cx + coil_R + 0.6, cy, cz_base + coil_height + 0.3)
ax.text(pt_curl[0], pt_curl[1], r"$I(t)$", fontsize=12,
        color=COIL_COLOR, fontweight="bold", ha="left", va="center", zorder=15)

# Coil label
pt_coil_lbl = p(cx - coil_R - 0.3, cy, cz_base + coil_height * 0.5)
ax.text(pt_coil_lbl[0], pt_coil_lbl[1], "EMAT\nCoil", fontsize=8,
        color=COIL_COLOR, ha="right", va="center",
        fontstyle="italic", zorder=15, fontfamily="serif")

# ══════════════════════════════════════════════════════════════
# 3. PERMANENT MAGNET (bar magnet beside coil)
# ══════════════════════════════════════════════════════════════
mx, my, mz = 2.8, 0.8, bz + 0.9
mw, md, mh = 0.4, 0.4, 1.8

# Magnet faces
p_polygon([(mx, my, mz), (mx + mw, my, mz),
           (mx + mw, my + md, mz), (mx, my + md, mz)],
          color=MAGNET_RED, alpha=0.7, zorder=11)
p_polygon([(mx, my, mz), (mx + mw, my, mz),
           (mx + mw, my, mz + mh), (mx, my, mz + mh)],
          color=MAGNET_RED, alpha=0.55, zorder=11)
p_polygon([(mx + mw, my, mz), (mx + mw, my + md, mz),
           (mx + mw, my + md, mz + mh), (mx + mw, my, mz + mh)],
          color=MAGNET_BLUE, alpha=0.55, zorder=11)

# N/S labels
pt_n = p(mx + mw / 2, my + md / 2, mz + mh + 0.2)
ax.text(pt_n[0], pt_n[1], "N", fontsize=11, fontweight="bold",
        color=MAGNET_RED, ha="center", va="bottom", zorder=15)
pt_s = p(mx + mw / 2, my + md / 2, mz - 0.2)
ax.text(pt_s[0], pt_s[1], "S", fontsize=11, fontweight="bold",
        color=MAGNET_BLUE, ha="center", va="top", zorder=15)

# Magnet label
pt_mag_lbl = p(mx + mw + 0.5, my + md / 2, mz + mh / 2)
ax.text(pt_mag_lbl[0], pt_mag_lbl[1], "Permanent\nMagnet",
        fontsize=8, color="#5D4037", ha="left", va="center",
        fontstyle="italic", zorder=15, fontfamily="serif")

# B-field arrows (static field from magnet, downward through material)
for bx_off in [0.0, 0.8, 1.6]:
    for by_off in [0.0, 0.8]:
        bstart_x = mx - 0.3 + bx_off
        bstart_y = my - 0.3 + by_off
        bstart_z = mz - 0.4
        p_arrow(bstart_x, bstart_y, bstart_z, 0, 0, -0.7,
                color=FIELD_COLOR, lw=1.2, zorder=9,
                shrinkA=0, shrinkB=2)

# B-field label
pt_b = p(mx - 0.8, my - 0.5, mz + 0.3)
ax.text(pt_b[0], pt_b[1], r"$\mathbf{B}$", fontsize=14,
        color=FIELD_COLOR, fontweight="bold", ha="center", va="center", zorder=15)

# ══════════════════════════════════════════════════════════════
# 4. EDDY CURRENTS (dense arc lines in material near surface)
# ══════════════════════════════════════════════════════════════
eddy_cx, eddy_cy = cx, cy
eddy_z = bz - 0.15
for r_eddy in [0.4, 0.65, 0.9, 1.15]:
    t = np.linspace(0.2, 2 * np.pi - 0.2, 50)
    ex = eddy_cx + r_eddy * np.cos(t)
    ey = eddy_cy + r_eddy * np.sin(t)
    ez = eddy_z * np.ones_like(t)
    pts = [p(x, y, z) for x, y, z in zip(ex, ey, ez)]
    sx = [pt[0] for pt in pts]
    sy = [pt[1] for pt in pts]
    alpha = 0.3 + 0.5 * (1 - r_eddy / 1.3)
    ax.plot(sx, sy, color=EDDY_COLOR, lw=1.2, alpha=alpha,
            solid_capstyle="round", zorder=6)

# Eddy current direction arrow (on one arc)
t_arr = np.linspace(1.0, 2.0, 20)
r_arr = 0.65
eax = eddy_cx + r_arr * np.cos(t_arr)
eay = eddy_cy + r_arr * np.sin(t_arr)
eaz = eddy_z * np.ones_like(t_arr)
pts_ea = [p(x, y, z) for x, y, z in zip(eax, eay, eaz)]
ax.annotate("", xy=pts_ea[-1], xytext=pts_ea[-4],
            arrowprops=dict(arrowstyle="-|>", color=EDDY_COLOR,
                            lw=1.5, mutation_scale=10), zorder=7)

# Eddy label
pt_eddy = p(eddy_cx + 1.5, eddy_cy - 0.8, eddy_z + 0.1)
ax.text(pt_eddy[0], pt_eddy[1], "Eddy\ncurrents",
        fontsize=8, color=EDDY_COLOR, ha="left", va="center",
        fontstyle="italic", zorder=15, fontfamily="serif")

# ══════════════════════════════════════════════════════════════
# 5. LORENTZ FORCE VECTORS (inside material, colored arrows)
# ══════════════════════════════════════════════════════════════
force_positions = [
    (0.8, 0.8, 0.5), (1.5, 1.5, 0.4), (2.2, 1.0, 0.6),
    (1.0, 2.0, 0.3), (1.8, 2.5, 0.5), (2.5, 2.0, 0.4),
    (1.2, 0.5, 0.7), (2.0, 0.5, 0.5), (0.5, 1.5, 0.6),
]
force_dirs = [
    (0, 0, 0.6), (0, 0, 0.5), (0, 0, 0.55),
    (0, 0, 0.5), (0, 0, 0.45), (0, 0, 0.5),
    (0, 0, 0.55), (0, 0, 0.5), (0, 0, 0.5),
]
for (fx, fy, fz), (fdx, fdy, fdz) in zip(force_positions, force_dirs):
    intensity = 1.0 - fz / 1.5
    col = plt.cm.YlOrBr(0.3 + 0.5 * intensity)
    p_arrow(fx, fy, fz, fdx, fdy, fdz,
            color=col, lw=1.5, zorder=8,
            shrinkA=1, shrinkB=3)

# Lorentz force label
pt_lorentz = p(0.3, 0.3, 0.2)
ax.text(pt_lorentz[0], pt_lorentz[1], r"$\mathbf{L} = \mathbf{J} \times \mathbf{B}$",
        fontsize=10, color=FORCE_ORANGE, fontweight="bold",
        ha="left", va="center", zorder=15)

# ══════════════════════════════════════════════════════════════
# 6. LIFT-OFF & TANGENTIAL DRIFT (dashed connection lines)
# ══════════════════════════════════════════════════════════════
# Lift-off arrow (vertical, beside coil)
lift_x, lift_y = cx - coil_R - 0.8, cy
lift_z_bot = bz
lift_z_top = bz + 1.0
p_line(lift_x, lift_y, lift_z_bot, lift_x, lift_y, lift_z_top,
       color=DASHED, lw=1.2, ls="--", zorder=14)
p_arrow(lift_x, lift_y, lift_z_top, 0, 0, 0.3,
        color=DASHED, lw=1.0, zorder=14, shrinkA=0, shrinkB=0)
pt_lift = p(lift_x - 0.3, lift_y, (lift_z_bot + lift_z_top) / 2)
ax.text(pt_lift[0], pt_lift[1], "Lift-off\n" + r"$h$",
        fontsize=8, color=DASHED, ha="right", va="center",
        fontfamily="serif", zorder=15)

# Dashed line from lift-off to eddy current region
eddy_connect_x, eddy_connect_y = eddy_cx - 0.2, eddy_cy
pt_lift_2d = p(lift_x, lift_y, lift_z_bot)
pt_eddy_2d = p(eddy_connect_x, eddy_connect_y, eddy_z + 0.3)
ax.plot([pt_lift_2d[0], pt_eddy_2d[0]], [pt_lift_2d[1], pt_eddy_2d[1]],
        color=DASHED, lw=0.8, ls=":", alpha=0.6, zorder=14)

# Tangential drift arrow (horizontal, on surface)
drift_x, drift_y, drift_z = bx + 0.3, by * 0.5, bz + 0.1
p_arrow(drift_x, drift_y, drift_z, 0.8, 0, 0,
        color=DASHED, lw=1.2, zorder=14, shrinkA=0, shrinkB=0)
pt_drift = p(drift_x + 0.4, drift_y, drift_z + 0.4)
ax.text(pt_drift[0], pt_drift[1], "Tangential\ndrift",
        fontsize=8, color=DASHED, ha="center", va="bottom",
        fontfamily="serif", zorder=15)

# Dashed line from drift to eddy current region
pt_drift_2d = p(drift_x, drift_y, drift_z)
pt_eddy_2d_2 = p(eddy_cx + 0.8, eddy_cy, eddy_z + 0.2)
ax.plot([pt_drift_2d[0], pt_eddy_2d_2[0]],
        [pt_drift_2d[1], pt_eddy_2d_2[1]],
        color=DASHED, lw=0.8, ls=":", alpha=0.6, zorder=14)

# ══════════════════════════════════════════════════════════════
# 7. FORMULA CARDS (bottom)
# ══════════════════════════════════════════════════════════════
formulas = [
    (r"$\mathbf{L} = \mathbf{J} \times \mathbf{B}$",
     "Lorentz body force"),
    (r"$\mathbf{J} = \sigma\!\left(\mathbf{E} + \mathbf{v} \times \mathbf{B}\right)$",
     "Current density"),
    (r"$c_L = \sqrt{\frac{\lambda + 2\mu}{\rho}}$",
     "Longitudinal wave speed"),
]

card_w, card_h = 3.8, 1.2
card_y = 0.3
card_xs = [1.5, 6.1, 10.7]

for i, (formula, label) in enumerate(formulas):
    x0 = card_xs[i]
    rect = FancyBboxPatch(
        (x0, card_y), card_w, card_h,
        boxstyle="round,pad=0.15",
        facecolor=FORMULA_BG, edgecolor=FORMULA_EDGE,
        linewidth=0.8, zorder=16,
        transform=ax.transData,
        clip_on=False,
    )
    ax.add_patch(rect)

    # Formula text
    ax.text(x0 + card_w / 2, card_y + card_h * 0.62, formula,
            fontsize=13, ha="center", va="center",
            color=TEXT_COLOR, fontweight="bold", zorder=17,
            fontfamily="serif")

    # Label text
    ax.text(x0 + card_w / 2, card_y + card_h * 0.22, label,
            fontsize=7.5, ha="center", va="center",
            color=ANNOT_COLOR, fontstyle="italic", zorder=17,
            fontfamily="serif")

# ══════════════════════════════════════════════════════════════
# 8. WAVE PROPAGATION INDICATOR (inside material)
# ══════════════════════════════════════════════════════════════
wave_x, wave_y = 2.8, 2.8
for wz in [0.3, 0.6, 0.9]:
    # Small zigzag wave
    t_wave = np.linspace(0, 1.5, 30)
    wx = wave_x + t_wave * 0.0
    wy = wave_y + t_wave * 0.0
    wz_vals = wz + 0.08 * np.sin(t_wave * 8)
    pts_w = [p(x, y, z) for x, y, z in zip(wx, wy, wz_vals)]
    sx_w = [pt[0] for pt in pts_w]
    sy_w = [pt[1] for pt in pts_w]

# Downward propagation arrow
p_arrow(wave_x, wave_y, 0.9, 0, 0, -0.5,
        color=FIELD_COLOR, lw=1.0, zorder=8, shrinkA=0, shrinkB=2)
pt_wave = p(wave_x + 0.4, wave_y, 0.65)
ax.text(pt_wave[0], pt_wave[1], r"$c_L$",
        fontsize=9, color=FIELD_COLOR, ha="left", va="center",
        fontfamily="serif", zorder=15)

# ══════════════════════════════════════════════════════════════
# 9. TITLE
# ══════════════════════════════════════════════════════════════
ax.text(8.0, 8.7, "EMAT Electromagnetic–Elastic Coupling Mechanism",
        fontsize=14, fontweight="bold", ha="center", va="top",
        color=TEXT_COLOR, fontfamily="serif", zorder=20)

# ══════════════════════════════════════════════════════════════
# SAVE
# ══════════════════════════════════════════════════════════════
plt.savefig("figures/emat_schematic.svg", bbox_inches="tight", dpi=300)
plt.savefig("figures/emat_schematic.pdf", bbox_inches="tight", dpi=300)
plt.savefig("figures/emat_schematic.tiff", bbox_inches="tight", dpi=600)
plt.savefig("figures/emat_schematic.png", bbox_inches="tight", dpi=300)
print("Saved: emat_schematic.svg / .pdf / .tiff / .png")
plt.close()
