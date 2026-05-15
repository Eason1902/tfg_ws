import xml.etree.ElementTree as ET
import numpy as np


def world_to_grid(x, y, origin_x, origin_y, resolution):
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    return row, col


def create_grid_from_world(world_file, resolution=0.2, world_size=20.0):
    origin_x = -world_size / 2.0
    origin_y = -world_size / 2.0

    grid_size = int(world_size / resolution)
    grid = np.zeros((grid_size, grid_size), dtype=int)

    tree = ET.parse(world_file)
    root = tree.getroot()

    for model in root.findall(".//model"):
        name = model.get("name", "").lower()

        # Ignore floor / ground models
        if "ground" in name or "floor" in name:
            continue

        pose_tag = model.find("pose")
        if pose_tag is None:
            continue

        pose_values = pose_tag.text.strip().split()
        x = float(pose_values[0])
        y = float(pose_values[1])

        box = model.find(".//box/size")
        if box is None:
            continue

        size_values = box.text.strip().split()
        size_x = float(size_values[0])
        size_y = float(size_values[1])

        # Ignore very large boxes, normally floor/platform
        if size_x > 20 or size_y > 20:
            continue

        min_x = x - size_x / 2.0
        max_x = x + size_x / 2.0
        min_y = y - size_y / 2.0
        max_y = y + size_y / 2.0

        row_min, col_min = world_to_grid(min_x, min_y, origin_x, origin_y, resolution)
        row_max, col_max = world_to_grid(max_x, max_y, origin_x, origin_y, resolution)

        row_min = max(0, row_min)
        row_max = min(grid_size - 1, row_max)
        col_min = max(0, col_min)
        col_max = min(grid_size - 1, col_max)

        grid[row_min:row_max + 1, col_min:col_max + 1] = 1

    return grid