#!/usr/bin/env python3
"""
generate_map.py
Defined Robotics — one-shot script to regenerate room_10x10.pgm

Run from any directory:
  python3 generate_map.py

Produces room_10x10.pgm in the same directory as this script.

Map specification:
  Room    : 10.0 m x 10.0 m
  Walls   : 2 cells (0.10 m) thick on all four sides
  Resolution: 0.05 m/pixel  →  200 x 200 pixels
  Free cells: 254 (white)
  Occupied  : 0   (black)
  Unknown   : 205 (grey) — unused in this static map
"""

import pathlib
import struct

RESOLUTION = 0.05        # m/pixel
ROOM_M     = 10.0        # metres
WALL_CELLS = 2           # cells thick

SIZE = int(ROOM_M / RESOLUTION)   # 200
FREE = 254
OCC  = 0

grid = [[FREE] * SIZE for _ in range(SIZE)]

# Mark wall cells occupied
for row in range(SIZE):
    for col in range(SIZE):
        if row < WALL_CELLS or row >= SIZE - WALL_CELLS:
            grid[row][col] = OCC
        elif col < WALL_CELLS or col >= SIZE - WALL_CELLS:
            grid[row][col] = OCC

out_path = pathlib.Path(__file__).parent / "room_10x10.pgm"
with open(out_path, "wb") as f:
    # PGM P5 header
    header = f"P5\n{SIZE} {SIZE}\n255\n"
    f.write(header.encode("ascii"))
    # Binary pixel data — row 0 is the top of the image.
    # ROS map_server convention: row 0 = max Y (north), increasing rows = south.
    for row in grid:
        f.write(bytes(row))

print(f"Written {out_path}  ({SIZE}x{SIZE} pixels, {RESOLUTION} m/px)")
