import matplotlib.pyplot as plt
from shapely.geometry import Polygon,Point
import numpy as np
import random

start = np.array([100.0, 100.0])
goal = np.array([1250.0, 1400.0])

def makeRectangle(existing_polygons):
    while True:
        width = random.uniform(50, 200)
        height = width * 1.5
        x = random.uniform(0, 1500)
        y = random.uniform(0, 1500)

        vertices = [(x, y), (x + width, y), (x + width, y + height), (x, y + height)]
        new_polygon = Polygon(vertices)

        # Check for intersection with existing polygons
        intersects = any(new_polygon.intersects(p) for p in existing_polygons)
        intersects_start = new_polygon.intersects(Point(start))
        intersects_end = new_polygon.intersects(Point(goal))

        if not intersects:
            existing_polygons.append(new_polygon)
            x, y = new_polygon.exterior.xy
            plt.plot(x, y)
            plt.fill(x, y, 'b')  
            break

def create_grid():
    grid_size = 1500
    cell_size = 1
    grid = np.zeros((grid_size, grid_size), dtype=bool)
    return grid

def mark_obstacles_in_grid(grid, existing_polygons):
    for polygon in existing_polygons:
        min_x, min_y, max_x, max_y = polygon.bounds
        min_x = max(0, int(min_x))
        min_y = max(0, int(min_y))
        max_x = min(grid.shape[1], int(max_x))
        max_y = min(grid.shape[0], int(max_y))
        
        grid[min_y:max_y, min_x:max_x] = True

    return grid

existing_polygons = []

def generate_obstacle():
    for _ in range(random.randint(8, 25)):
        makeRectangle(existing_polygons)

    plt.axis('off')
    plt.gca().set_aspect('equal', adjustable='box')  
    plt.savefig('obstacle_plot.png', bbox_inches='tight', pad_inches=0, transparent=True)

    grid = create_grid()
    grid = mark_obstacles_in_grid(grid, existing_polygons)
    np.save('obstacle_grid.npy', grid)
