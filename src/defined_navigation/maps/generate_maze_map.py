#!/usr/bin/env python3
"""
generate_maze_map.py
Defined Robotics — one-shot script to regenerate maze_10x10.pgm

Run from any directory:
  python3 generate_maze_map.py

Produces maze_10x10.pgm in the same directory as this script.

Map specification:
  Room      : 10.0 m x 10.0 m, centred at (0,0)
  Resolution: 0.05 m/pixel  →  200 x 200 pixels
  Origin    : (-5.0, -5.0)  (bottom-left corner)
  Free cells: 254 (white)
  Occupied  : 0   (black)

  Outer walls: 2 cells (0.10 m) thick on all four sides

  Internal maze walls (S-curve):
    Wall A: y = +2.0 m, x = -5.0 to +1.0 m  (gap on east side)
    Wall B: y = -2.0 m, x = -1.0 to +5.0 m  (gap on west side)
    Both walls: 0.20 m thick → 4 pixel rows, centred on the wall Y coordinate.

  Coordinate → pixel mapping (PGM row 0 = top = north = y=+5):
    col = int((x + 5.0) / RESOLUTION)
    row = int((5.0 - y) / RESOLUTION)
"""

import pathlib

RESOLUTION = 0.05   # m/pixel
ROOM_M     = 10.0   # metres
SIZE       = int(ROOM_M / RESOLUTION)   # 200
FREE       = 254
OCC        = 0


def world_to_pixel(x_m, y_m):
    """Convert world coordinates (metres) to (col, row) pixel indices."""
    col = int((x_m + 5.0) / RESOLUTION)
    row = int((5.0 - y_m) / RESOLUTION)
    return col, row


grid = [[FREE] * SIZE for _ in range(SIZE)]

# ---------------------------------------------------------------------------
# Outer perimeter walls (2 cells = 0.10 m thick)
# ---------------------------------------------------------------------------
OUTER = 2
for r in range(SIZE):
    for c in range(SIZE):
        if r < OUTER or r >= SIZE - OUTER or c < OUTER or c >= SIZE - OUTER:
            grid[r][c] = OCC

# ---------------------------------------------------------------------------
# Internal wall helper: mark a horizontal band of cells occupied
#   y_m   : wall centre Y in world coordinates
#   x0_m  : wall start X in world coordinates (inclusive)
#   x1_m  : wall end   X in world coordinates (inclusive)
#   thick  : wall thickness in metres (default 0.20 m)
# ---------------------------------------------------------------------------
def draw_h_wall(y_m, x0_m, x1_m, thick=0.20):
    half = thick / 2.0
    y_min, y_max = y_m - half, y_m + half
    # row numbers span from the northern edge (smaller row) to southern (larger row)
    row_min = int((5.0 - y_max) / RESOLUTION)
    row_max = int((5.0 - y_min) / RESOLUTION)
    col_min = int((x0_m + 5.0) / RESOLUTION)
    col_max = int((x1_m + 5.0) / RESOLUTION)
    for r in range(max(0, row_min), min(SIZE, row_max + 1)):
        for c in range(max(0, col_min), min(SIZE, col_max + 1)):
            grid[r][c] = OCC


# Wall A: y=+2.0, x=-5.0 to x=+1.0 (gap from x=+1.0 to x=+5.0 on east side)
draw_h_wall(y_m=2.0, x0_m=-5.0, x1_m=1.0)

# Wall B: y=-2.0, x=-1.0 to x=+5.0 (gap from x=-5.0 to x=-1.0 on west side)
draw_h_wall(y_m=-2.0, x0_m=-1.0, x1_m=5.0)

# ---------------------------------------------------------------------------
# Write PGM
# ---------------------------------------------------------------------------
out_path = pathlib.Path(__file__).parent / "maze_10x10.pgm"
with open(out_path, "wb") as f:
    header = f"P5\n{SIZE} {SIZE}\n255\n"
    f.write(header.encode("ascii"))
    for row in grid:
        f.write(bytes(row))

print(f"Written {out_path}  ({SIZE}x{SIZE} pixels, {RESOLUTION} m/px)")
print(f"  Wall A: y=+2.0 m, x=-5.0 to +1.0 m  (east gap)")
print(f"  Wall B: y=-2.0 m, x=-1.0 to +5.0 m  (west gap)")
