#!/usr/bin/env python3
"""
generate_warehouse_map.py
Defined Robotics — one-shot script to regenerate warehouse.pgm

Run from any directory:
  python3 generate_warehouse_map.py

Produces warehouse.pgm in the same directory as this script.

Map specification:
  Room      : 16.0 m (X) x 10.0 m (Y), centred at (0,0)
  Resolution: 0.05 m/pixel  →  320 x 200 pixels
  Origin    : (-8.0, -5.0)  (bottom-left corner)

  Outer walls: 2 cells (0.10 m) thick
  Shelf rows: upper at y=+2.0, lower at y=-2.0
              Each shelf: 4.0 m long, 0.4 m deep (8 px)
              Three shelves at x = -4.0, 0.0, +4.0
"""

import pathlib

RESOLUTION = 0.05   # m/pixel
ROOM_X     = 16.0   # metres
ROOM_Y     = 10.0
COLS       = int(ROOM_X / RESOLUTION)   # 320
ROWS       = int(ROOM_Y / RESOLUTION)   # 200
FREE       = 254
OCC        = 0

# Room centred at origin: x ∈ [-8, +8], y ∈ [-5, +5]
X_MIN = -ROOM_X / 2  # -8.0
Y_MAX =  ROOM_Y / 2  #  5.0


def world_to_pixel(x_m, y_m):
    col = int((x_m - X_MIN) / RESOLUTION)
    row = int((Y_MAX - y_m) / RESOLUTION)
    return col, row


grid = [[FREE] * COLS for _ in range(ROWS)]

# ---------------------------------------------------------------------------
# Outer walls (2 cells thick)
# ---------------------------------------------------------------------------
OUTER = 2
for r in range(ROWS):
    for c in range(COLS):
        if r < OUTER or r >= ROWS - OUTER or c < OUTER or c >= COLS - OUTER:
            grid[r][c] = OCC

# ---------------------------------------------------------------------------
# Shelf helper: rectangular obstacle
# ---------------------------------------------------------------------------
def draw_rect(cx_m, cy_m, width_m, depth_m):
    """Draw a filled rectangle centred at (cx, cy) in world coordinates."""
    x0 = cx_m - width_m / 2
    x1 = cx_m + width_m / 2
    y0 = cy_m - depth_m / 2
    y1 = cy_m + depth_m / 2
    row_min = int((Y_MAX - y1) / RESOLUTION)
    row_max = int((Y_MAX - y0) / RESOLUTION)
    col_min = int((x0 - X_MIN) / RESOLUTION)
    col_max = int((x1 - X_MIN) / RESOLUTION)
    for r in range(max(0, row_min), min(ROWS, row_max + 1)):
        for c in range(max(0, col_min), min(COLS, col_max + 1)):
            grid[r][c] = OCC


# Upper shelf row (y = +2.0): shelves A, B, C
for sx in [-4.0, 0.0, 4.0]:
    draw_rect(sx, 2.0, 4.0, 0.4)

# Lower shelf row (y = -2.0): shelves D, E, F
for sx in [-4.0, 0.0, 4.0]:
    draw_rect(sx, -2.0, 4.0, 0.4)

# ---------------------------------------------------------------------------
# Write PGM
# ---------------------------------------------------------------------------
out_path = pathlib.Path(__file__).parent / "warehouse.pgm"
with open(out_path, "wb") as f:
    header = f"P5\n{COLS} {ROWS}\n255\n"
    f.write(header.encode("ascii"))
    for row in grid:
        f.write(bytes(row))

print(f"Written {out_path}  ({COLS}x{ROWS} pixels, {RESOLUTION} m/px)")
