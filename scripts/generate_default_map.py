#!/usr/bin/env python3
"""
Generate the default occupancy grid map matching worlds/warehouse_nav.sdf.

Produces warehouse_default.pgm and warehouse_default.yaml in the maps directory.
"""
import math
from pathlib import Path
import numpy as np
from PIL import Image

def generate_warehouse_map(
    output_dir: Path,
    resolution: float = 0.05,
    origin_x: float = -15.5,
    origin_y: float = -10.5,
    width_m: float = 31.0,
    height_m: float = 21.0,
):
    width_px = int(math.ceil(width_m / resolution))
    height_px = int(math.ceil(height_m / resolution))

    # Initialize with unknown (205)
    grid = np.full((height_px, width_px), 205, dtype=np.uint8)

    def world_to_px(x, y):
        col = int((x - origin_x) / resolution)
        # In PGM image, row 0 is top (max y), row (height_px-1) is bottom (min y)
        row = height_px - 1 - int((y - origin_y) / resolution)
        return row, col

    # Free interior: x in [-15, 15], y in [-10, 10]
    r_min, c_min = world_to_px(-15.0, 10.0)
    r_max, c_max = world_to_px(15.0, -10.0)
    grid[r_min:r_max+1, c_min:c_max+1] = 254

    # Helper to mark rectangular obstacle
    def add_box(cx, cy, sx, sy, yaw=0.0):
        # Sample points inside the oriented box
        cos_y = math.cos(-yaw)
        sin_y = math.sin(-yaw)
        hw = sx / 2.0
        hh = sy / 2.0
        
        # Bounding box in world coordinates
        diag = math.hypot(hw, hh)
        x_min = cx - diag
        x_max = cx + diag
        y_min = cy - diag
        y_max = cy + diag

        r_top, c_left = world_to_px(x_min, y_max)
        r_bottom, c_right = world_to_px(x_max, y_min)

        r_top = max(0, min(height_px - 1, r_top))
        r_bottom = max(0, min(height_px - 1, r_bottom))
        c_left = max(0, min(width_px - 1, c_left))
        c_right = max(0, min(width_px - 1, c_right))

        for r in range(r_top, r_bottom + 1):
            y_w = origin_y + (height_px - 1 - r) * resolution
            for c in range(c_left, c_right + 1):
                x_w = origin_x + c * resolution
                # Transform to local frame
                dx = x_w - cx
                dy = y_w - cy
                lx = dx * cos_y - dy * sin_y
                ly = dx * sin_y + dy * cos_y
                if abs(lx) <= hw and abs(ly) <= hh:
                    grid[r, c] = 0

    # Helper to mark cylinder obstacle
    def add_cylinder(cx, cy, radius):
        r_top, c_left = world_to_px(cx - radius, cy + radius)
        r_bottom, c_right = world_to_px(cx + radius, cy - radius)

        r_top = max(0, min(height_px - 1, r_top))
        r_bottom = max(0, min(height_px - 1, r_bottom))
        c_left = max(0, min(width_px - 1, c_left))
        c_right = max(0, min(width_px - 1, c_right))

        for r in range(r_top, r_bottom + 1):
            y_w = origin_y + (height_px - 1 - r) * resolution
            for c in range(c_left, c_right + 1):
                x_w = origin_x + c * resolution
                if math.hypot(x_w - cx, y_w - cy) <= radius:
                    grid[r, c] = 0

    # 1. Perimeter walls
    add_box(0.0, 10.0, 30.0, 0.15)    # North
    add_box(0.0, -10.0, 30.0, 0.15)   # South
    add_box(15.0, 0.0, 0.15, 20.0)    # East
    add_box(-15.0, 0.0, 0.15, 20.0)   # West

    # 2. Interior dividers
    add_box(-5.0, 0.0, 0.15, 8.0)     # Divider left
    add_box(-5.0, 7.0, 0.15, 6.0)     # Divider right
    add_box(5.0, 3.0, 10.0, 0.15)     # Corridor wall

    # 3. Shelves
    add_box(-10.0, -5.0, 3.0, 0.6)
    add_box(-10.0, -2.0, 3.0, 0.6)
    add_box(-10.0, 1.0, 3.0, 0.6)
    add_box(5.0, -5.0, 4.0, 0.6)
    add_box(5.0, -2.0, 4.0, 0.6)
    add_box(10.0, 6.0, 3.0, 0.6, yaw=math.pi / 2.0)

    # 4. Scattered boxes
    add_box(-2.0, -7.0, 0.6, 0.6, yaw=0.3)
    add_box(2.0, 7.0, 0.5, 0.5, yaw=-0.2)
    add_box(8.0, -8.0, 0.7, 0.7, yaw=0.8)

    # 5. Pillars
    add_cylinder(0.0, 0.0, 0.20)
    add_cylinder(10.0, 0.0, 0.20)

    # Save PGM image
    output_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = output_dir / "warehouse_default.pgm"
    img = Image.fromarray(grid, mode="L")
    img.save(str(pgm_path))

    # Save YAML file
    yaml_path = output_dir / "warehouse_default.yaml"
    yaml_content = f"""image: warehouse_default.pgm
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
mode: trinary
"""
    yaml_path.write_text(yaml_content)
    print(f"Generated map: {pgm_path} ({width_px}x{height_px}) and {yaml_path}")


if __name__ == "__main__":
    out_dir = Path(__file__).resolve().parent.parent / "maps"
    generate_warehouse_map(out_dir)
