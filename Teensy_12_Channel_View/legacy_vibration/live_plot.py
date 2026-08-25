import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import math
import serial.tools.list_ports

# ==========================
# Serial settings
# ==========================

# Generalized port
def find_teensy_port():
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in ports:
        if 'Teensy' in desc or 'USB' in desc:  # Adjust filter as needed
            return port
    raise Exception("No Teensy found!")

PORT = find_teensy_port()

# PORT  = "COM3"

BAUD  = 115200 #Speed of serial communication
W     = 500    #Time window we look at (last # samples)

# ==========================
# Exciter drive frequency
# ==========================
FREQ  = 50       # Hz — change to your (Function Generator) SDG6022X frequency
G     = 9.81      # gravitational acceleration in m/s²
omega = 2 * math.pi * FREQ

#Define a function to convert acceleration to displacement
def accel_to_displacement_mm(rms_g):
    return (rms_g * G / (omega ** 2)) * 1000 #d=a/(2*pi*f)^2

ser = serial.Serial(PORT, BAUD, timeout=10) #Wait for # seconds before giving up
ser.reset_input_buffer()                    #Clear buffer before excuting the script

# ==========================
# Data buffers
# ==========================
rms_x  = deque([0.0] * W, maxlen=W)
rms_y  = deque([0.0] * W, maxlen=W)
rms_z  = deque([0.0] * W, maxlen=W)
disp_x = deque([0.0] * W, maxlen=W)
disp_y = deque([0.0] * W, maxlen=W)
disp_z = deque([0.0] * W, maxlen=W)

# ==========================
# Plot setup — 2 subplots
# ==========================
fig, (ax_rms, ax_disp) = plt.subplots(2, 1, figsize=(9, 7))

# ── RMS plot ─────────────────────────────────
line_rx, = ax_rms.plot(range(W), list(rms_x), label="X (g)")
line_ry, = ax_rms.plot(range(W), list(rms_y), label="Y (g)")
line_rz, = ax_rms.plot(range(W), list(rms_z), label="Z (g)")

# Use a clean publication style
plt.style.use("seaborn-v0_8-whitegrid")

# Set global font to match IEEE papers
plt.rcParams.update({
    "font.family"      : "Times New Roman",
    "font.size"        : 11,
    "axes.titlesize"   : 12,
    "axes.labelsize"   : 11,
    "xtick.labelsize"  : 10,
    "ytick.labelsize"  : 10,
    "legend.fontsize"  : 10,
    "figure.dpi"       : 300,
    "lines.linewidth"  : 1.5,
})

# Then for your axes:
ax_rms.set_title("Acceleration — X, Y, Z", fontweight="bold", pad=10)
ax_rms.set_xlabel("Windows (512 Samples Each)", labelpad=8)
ax_rms.set_ylabel("Acceleration (g)", labelpad=8)
ax_rms.set_ylim(-1, 1)

# Cleaner legend
ax_rms.legend(
    loc="upper left",
    frameon=True,
    framealpha=0.9,
    edgecolor="gray",
    fancybox=False
)

# Cleaner grid
ax_rms.grid(True, linestyle="--", linewidth=0.5, alpha=0.7)

# Thicker axis border
for spine in ax_rms.spines.values():
    spine.set_linewidth(1.2)

# Tick marks inside
ax_rms.tick_params(
    direction="in",
    top=True,
    right=True,
    length=4
)

# Set figure size to IEEE column width (3.5 inch single, 7.2 inch double)
fig.set_size_inches(7.2, 5)

# ── Displacement plot ─────────────────────────
line_dx, = ax_disp.plot(range(W), list(disp_x), label="Disp X (mm)")
line_dy, = ax_disp.plot(range(W), list(disp_y), label="Disp Y (mm)")
line_dz, = ax_disp.plot(range(W), list(disp_z), label="Disp Z (mm)")

ax_disp.set_title(f"Displacement — X, Y, Z  (f = {FREQ} Hz)", 
                  fontweight="bold", pad=10)
ax_disp.set_xlabel("Windows (512 Samples Each)", labelpad=8)
ax_disp.set_ylabel("Displacement (mm)", labelpad=8)
ax_disp.set_ylim(0, 1)

ax_disp.legend(
    loc="upper left",
    frameon=True,
    framealpha=0.9,
    edgecolor="gray",
    fancybox=False
)

ax_disp.grid(True, linestyle="--", linewidth=0.5, alpha=0.7)

for spine in ax_disp.spines.values():
    spine.set_linewidth(1.2)

ax_disp.tick_params(
    direction="in",
    top=True,
    right=True,
    length=4
)

plt.tight_layout()
#Reactive arrow
tooltip_rms  = ax_rms.annotate("",
    xy=(0, 0), xytext=(15, 15),
    textcoords="offset points",
    bbox=dict(boxstyle="round,pad=0.4", fc="white", ec="gray", lw=0.8),
    arrowprops=dict(arrowstyle="->", color="gray"),
    fontsize=9,
    visible=False
)

tooltip_disp = ax_disp.annotate("",
    xy=(0, 0), xytext=(15, 15),
    textcoords="offset points",
    bbox=dict(boxstyle="round,pad=0.4", fc="white", ec="gray", lw=0.8),
    arrowprops=dict(arrowstyle="->", color="gray"),
    fontsize=9,
    visible=False
)

def on_hover(event):
    if event.inaxes == ax_rms:
        x = int(round(event.xdata)) if event.xdata is not None else None
        if x is not None and 0 <= x < W:
            rx = list(rms_x)[x]
            ry = list(rms_y)[x]
            rz = list(rms_z)[x]
            tooltip_rms.set_visible(True)
            tooltip_rms.xy = (x, max(rx, ry, rz))
            tooltip_rms.set_text(
                f"Window : {x}\n"
                f"X : {rx:.6f} g\n"
                f"Y : {ry:.6f} g\n"
                f"Z : {rz:.6f} g"
            )
        else:
            tooltip_rms.set_visible(False)
        fig.canvas.draw_idle()

    elif event.inaxes == ax_disp:
        x = int(round(event.xdata)) if event.xdata is not None else None
        if x is not None and 0 <= x < W:
            dx = list(disp_x)[x]
            dy = list(disp_y)[x]
            dz = list(disp_z)[x]
            tooltip_disp.set_visible(True)
            tooltip_disp.xy = (x, max(dx, dy, dz))
            tooltip_disp.set_text(
                f"Window : {x}\n"
                f"X : {dx:.6f} mm\n"
                f"Y : {dy:.6f} mm\n"
                f"Z : {dz:.6f} mm"
            )
        else:
            tooltip_disp.set_visible(False)
        fig.canvas.draw_idle()

    else:
        tooltip_rms.set_visible(False)
        tooltip_disp.set_visible(False)
        fig.canvas.draw_idle()

fig.canvas.mpl_connect("motion_notify_event", on_hover)
#Snapshot saving
def on_key(event):
    if event.key == "s":
        fig.savefig("vibration_snapshot.pdf", bbox_inches="tight", dpi=300)
        print("Snapshot saved!")

fig.canvas.mpl_connect("key_press_event", on_key)
# ==========================
# Update function
# ==========================
def update(frame):
    try:
        line = ser.readline().decode(errors="ignore").strip()

        if not line or "," not in line:
            return line_rx, line_ry, line_rz, line_dx, line_dy, line_dz

        values = line.split(",")
        if len(values) == 3:

            # RMS values
            rx = float(values[0])
            ry = float(values[1])
            rz = float(values[2])

            rms_x.append(rx)
            rms_y.append(ry)
            rms_z.append(rz)

            # Displacement values
            disp_x.append(accel_to_displacement_mm(rx))
            disp_y.append(accel_to_displacement_mm(ry)) #d=a/(2*pi*f)^2
            disp_z.append(accel_to_displacement_mm(rz))

            # Update RMS lines
            line_rx.set_ydata(list(rms_x))
            line_ry.set_ydata(list(rms_y))
            line_rz.set_ydata(list(rms_z))

            # Update displacement lines
            line_dx.set_ydata(list(disp_x))
            line_dy.set_ydata(list(disp_y))
            line_dz.set_ydata(list(disp_z))

            # Auto-scale RMS
            max_rms = max(max(rms_x), max(rms_y), max(rms_z)) * 1.3
            ax_rms.set_ylim(0, max(max_rms, 0.01))

            # Auto-scale displacement
            max_disp = max(max(disp_x), max(disp_y), max(disp_z)) * 1.3
            ax_disp.set_ylim(0, max(max_disp, 0.001))

    except:
        pass

    return line_rx, line_ry, line_rz, line_dx, line_dy, line_dz

# ==========================
# Run
# ==========================
ani = FuncAnimation(
    fig,
    update,
    interval=10,
    blit=True,
    cache_frame_data=False
)

plt.show()
ser.close()