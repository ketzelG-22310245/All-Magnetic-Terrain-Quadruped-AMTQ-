#!/usr/bin/env python3
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

csv_file = "learning_log.csv"
if not os.path.exists(csv_file):
    print(f"No se encontró '{csv_file}'. Asegúrate de correr primero amtq_gait_controller.py.")
    exit(1)

cols = ["t_s", "x_mm", "y_mm", "dx_step", "dy_step", "heading_deg", "yaw_err_deg", "yaw_trim"]
df = pd.read_csv(csv_file, names=cols)
df["cycle"] = np.arange(1, len(df) + 1)

# Posición relativa desde el punto de inicio (0, 0)
x_rel = df["x_mm"] - df["x_mm"].iloc[0]
y_rel = df["y_mm"] - df["y_mm"].iloc[0]

fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# 1. Trayectoria 2D en el plano XY
ax1 = axes[0, 0]
sc = ax1.scatter(x_rel, y_rel, c=df["cycle"], cmap="viridis", s=35, zorder=4, label="Posición por ciclo")
ax1.plot(x_rel, y_rel, color="#1f77b4", linewidth=2, alpha=0.7, zorder=3)
ax1.scatter([0], [0], color="blue", s=90, zorder=5, label="Inicio (0, 0)")
ax1.scatter([x_rel.iloc[-1]], [y_rel.iloc[-1]], color="red", s=110, marker="*", zorder=5,
            label=f"Fin ({x_rel.iloc[-1]:.1f}, {y_rel.iloc[-1]:.1f}) mm")
ax1.set_title("1. Trayectoria 2D del Robot en el Plano XY", fontweight="bold")
ax1.set_xlabel("Posición Transversal X (mm)")
ax1.set_ylabel("Posición Longitudinal Y (mm)")
ax1.axvline(0, color="gray", linestyle=":", linewidth=1)
ax1.axhline(0, color="gray", linestyle=":", linewidth=1)
ax1.set_aspect("equal", adjustable="datalim")
ax1.grid(True, linestyle="--", alpha=0.5)
ax1.legend(loc="best")
plt.colorbar(sc, ax=ax1, label="Ciclo de Marcha (N)")

# 2. Error lateral (dX) y avance (dY) por ciclo
ax2 = axes[0, 1]
ax2.plot(df["cycle"], df["dy_step"], color="#2ca02c", marker=".", alpha=0.8, label="Avance dY / ciclo (mm)")
ax2.plot(df["cycle"], df["dx_step"], color="#d62728", marker=".", alpha=0.85, label="Desvío lateral dX / ciclo (mm)")
ax2.plot(df["cycle"], df["dx_step"].rolling(5, min_periods=1).mean(), color="darkred", linewidth=2.5,
         label="Media móvil dX (5 ciclos)")
ax2.axhline(0.0, color="black", linestyle="--", linewidth=1.2, label="Ideal (dX = 0.0 mm)")
ax2.set_title("2. Convergencia del Error Lateral por Paso (dX -> 0 mm)", fontweight="bold")
ax2.set_xlabel("Ciclo de Marcha (N)")
ax2.set_ylabel("Desplazamiento por Ciclo (mm)")
ax2.grid(True, linestyle="--", alpha=0.5)
ax2.legend(loc="best")

# 3. Aprendizaje del ángulo Heading
ax3 = axes[1, 0]
ax3.plot(df["cycle"], df["heading_deg"], color="#9467bd", marker="o", markersize=3, linewidth=2.2,
         label="Heading aprendido (°)")
ax3.set_title("3. Curva de Aprendizaje del Ángulo de Proyección (Heading)", fontweight="bold")
ax3.set_xlabel("Ciclo de Marcha (N)")
ax3.set_ylabel("Ángulo Heading (grados)")
ax3.grid(True, linestyle="--", alpha=0.5)
ax3.legend(loc="best")

# 4. Error de Giro (YawErr) y YawTrim
ax4 = axes[1, 1]
ax4.plot(df["cycle"], df["yaw_err_deg"], color="#ff7f0e", linewidth=2.2, label="Error de Giro YawErr (°)")
ax4.axhline(0.0, color="gray", linestyle=":", linewidth=1)
ax4.set_title("4. Dinámica de Giro (YawErr) y Compensación Aprendida (YawTrim)", fontweight="bold")
ax4.set_xlabel("Ciclo de Marcha (N)")
ax4.set_ylabel("YawErr (grados)", color="#ff7f0e")
ax4.tick_params(axis="y", labelcolor="#ff7f0e")
ax4.grid(True, linestyle="--", alpha=0.5)

ax4_t = ax4.twinx()
ax4_t.plot(df["cycle"], df["yaw_trim"], color="#17becf", linestyle="--", linewidth=2.2, label="YawTrim aprendido")
ax4_t.set_ylabel("YawTrim", color="#17becf")
ax4_t.tick_params(axis="y", labelcolor="#17becf")

l1, lb1 = ax4.get_legend_handles_labels()
l2, lb2 = ax4_t.get_legend_handles_labels()
ax4.legend(l1 + l2, lb1 + lb2, loc="best")

plt.tight_layout()
out_img = "trayectoria_y_aprendizaje.png"
plt.savefig(out_img, dpi=150)
print(f"Gráfica guardada exitosamente en: {os.path.abspath(out_img)}")
