def grid_to_world(row, col, origin_x, origin_y, resolution):
    x = origin_x + col * resolution
    y = origin_y + row * resolution
    return x, y


def path_to_world(path, world_size=30.0, resolution=0.2):
    origin_x = -world_size / 2.0
    origin_y = -world_size / 2.0

    world_path = []

    for col, row in path:
        x, y = grid_to_world(row, col, origin_x, origin_y, resolution)
        world_path.append((x, y))

    return world_path
