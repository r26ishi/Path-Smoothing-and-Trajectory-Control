#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt


# ============================================================
# CSV reader
# ============================================================

def read_path(filename):

    x = []
    y = []

    with open(filename, "r") as file:

        reader = csv.DictReader(file)

        for row in reader:
            x.append(float(row["x"]))
            y.append(float(row["y"]))

    return x, y


# ============================================================
# Read paths
# ============================================================


raw_x, raw_y = read_path("/home/hild/nav_ws/raw_path.csv")
smooth_x, smooth_y = read_path("/home/hild/nav_ws/smoothed_plan.csv")


# ============================================================
# Plot
# ============================================================

plt.figure(figsize=(10, 8))


# Raw path
plt.plot(
    raw_x,
    raw_y,
    "o--",
    linewidth=2,
    markersize=7,
    label="Raw Path"
)


# Smoothed path
plt.plot(
    smooth_x,
    smooth_y,
    "-",
    linewidth=2.5,
    label="Smoothed Plan"
)


# ============================================================
# Start point
# ============================================================

plt.scatter(
    raw_x[0],
    raw_y[0],
    s=120,
    marker="o",
    label="Start",
    zorder=5
)


# ============================================================
# Goal point
# ============================================================

plt.scatter(
    raw_x[-1],
    raw_y[-1],
    s=150,
    marker="x",
    label="Goal",
    zorder=5
)


# ============================================================
# Labels
# ============================================================

plt.xlabel("X [m]")
plt.ylabel("Y [m]")

plt.title("Raw Path vs Smoothed Plan")

plt.grid(True)

# Keep X/Y scale equal
plt.axis("equal")

plt.legend()

plt.tight_layout()

plt.show()
