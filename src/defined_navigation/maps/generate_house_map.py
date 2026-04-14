#!/usr/bin/env python3
"""
generate_house_map.py
Defined Robotics — one-shot script to regenerate house.pgm

Run from any directory:
  python3 generate_house_map.py

Produces house.pgm in the same directory as this script.

Map specification:
  Room      : 10.0 m (X) x 8.0 m (Y), centred at (0,0)
  Resolution: 0.05 m/pixel  →  200 x 160 pixels
  Origin    : (-5.0, -4.0)  (bottom-left corner)

  Outer walls: 2 cells thick (0.10 m)
  Interior walls: 3 cells thick (0.15 m)

  Layout:
    Central N-S wall at x=0, full height, doorways at y=+1.0 and y=-1.0
    Left E-W wall at y=0 (x=-5 to 0), doorway at x=-1.0
    Right E-W wall at y=-1 (x=0 to +5), doorway at x=+2.0

  Rooms:
    Bedroom    (-3, +2)  — top-left
    Living Room (+3, +2) — top-right
    Kitchen    (-3, -2)  — bottom-left
    Entrance   (+3, -2)  — bottom-right
"""

import pathlib

RESOLUTION = 0.05   # m/pixel
ROOM_X     = 10.0   # metres
ROOM_Y     = 8.0
COLS       = int(ROOM_X / RESOLUTION)   # 200
ROWS       = int(ROOM_Y / RESOLUTION)   # 160
FREE       = 254
OCC        = 0

X_MIN = -ROOM_X / 2  # -5.0
Y_MAX =  ROOM_Y / 2  #  4.0


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
# Wall helpers
# ---------------------------------------------------------------------------
WALL_THICK = 0.15  # interior wall thickness

def draw_h_wall(y_m, x0_m, x1_m, thick=WALL_THICK):
    """Horizontal wall (constant y)."""
    half = thick / 2.0
    row_min = int((Y_MAX - (y_m + half)) / RESOLUTION)
    row_max = int((Y_MAX - (y_m - half)) / RESOLUTION)
    col_min = int((x0_m - X_MIN) / RESOLUTION)
    col_max = int((x1_m - X_MIN) / RESOLUTION)
    for r in range(max(0, row_min), min(ROWS, row_max + 1)):
        for c in range(max(0, col_min), min(ROWS, col_max + 1)):
            grid[r][c] = OCC

def draw_v_wall(x_m, y0_m, y1_m, thick=WALL_THICK):
    """Vertical wall (constant x). y0 < y1."""
    half = thick / 2.0
    row_min = int((Y_MAX - y1_m) / RESOLUTION)
    row_max = int((Y_MAX - y0_m) / RESOLUTION)
    col_min = int((x_m - half - X_MIN) / RESOLUTION)
    col_max = int((x_m + half - X_MIN) / RESOLUTION)
    for r in range(max(0, row_min), min(ROWS, row_max + 1)):
        for c in range(max(0, col_min), min(COLS, col_max + 1)):
            grid[r][c] = OCC


# ---------------------------------------------------------------------------
# Central N-S wall at x=0, with 1.0 m doorways at y=+1.0 and y=-1.0
# ---------------------------------------------------------------------------
# Top section: y = +4.0 down to y = +1.5
draw_v_wall(0.0, 1.5, 4.0)
# Middle section: y = +0.5 down to y = -0.5
draw_v_wall(0.0, -0.5, 0.5)
# Bottom section: y = -1.5 down to y = -4.0
draw_v_wall(0.0, -4.0, -1.5)

# ---------------------------------------------------------------------------
# Left E-W wall at y=0 (bedroom / kitchen divider)
# Doorway at x = -1.0 (gap from -1.5 to -0.5)
# ---------------------------------------------------------------------------
# Western section: x = -5.0 to -1.5
draw_h_wall(0.0, -5.0, -1.5)
# Eastern stub: x = -0.5 to 0.0
draw_h_wall(0.0, -0.5, 0.0)

# ---------------------------------------------------------------------------
# Right E-W wall at y=-1.0 (living room / entrance divider)
# Doorway at x = +2.0 (gap from +1.5 to +2.5)
# ---------------------------------------------------------------------------
# Western section: x = 0.0 to +1.5
draw_h_wall(-1.0, 0.0, 1.5)
# Eastern section: x = +2.5 to +5.0
draw_h_wall(-1.0, 2.5, 5.0)

# ---------------------------------------------------------------------------
# Write PGM
# ---------------------------------------------------------------------------
out_path = pathlib.Path(__file__).parent / "house.pgm"
with open(out_path, "wb") as f:
    header = f"P5\n{COLS} {ROWS}\n255\n"
    f.write(header.encode("ascii"))
    for row in grid:
        f.write(bytes(row))

print(f"Written {out_path}  ({COLS}x{ROWS} pixels, {RESOLUTION} m/px)")
