import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import cos, sin, radians

"""

This started code provides code base to implement occupancy grid mapping for HW5 Q2.
This is a stand alone code and does not require ROS
--- TBD --- indicates the section to fill out   
... indicates the specific place to type the code
"""


class OccupancyGridMappingSim:
    def __init__(self):
        # Grid setup
        self.grid_size = 50
        self.center = (self.grid_size // 2, self.grid_size // 2)
        self.radius = 27  # LiDAR scan radius in pixel value

        # Log-odds values
        # --------------------------- TBD ------------------------
        self.l0 = ... # initial probability of all cells will be 0.5. convert this to log odds value
        self.log_occ = ...   # use higher positive values [0, 1] for occupied cell
        self.log_free = ...  # use smaller negative values [0, 1] for empty cells
        self.log_min = ...  # calculate log odds lower bounds for probability = 0.1
        self.log_max = ...  # calculate log odds upper bounds for probability = 0.9
        # ----------------------- TBD ENDS ------------------------

        # Initialize occupancy grid
        self.log_odds = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)
        self.obstacle_mask = np.zeros((self.grid_size, self.grid_size), dtype=bool)
        self.obstacle_mask[48:50, 15:35] = True  # static obstacle at top
        self.obstacle_mask[15:18, 15:18] = True  # static obstacle at bottom-left

        # Add borders as obstacles
        self.obstacle_mask[0, :] = True  # bottom row
        self.obstacle_mask[-1, :] = True  # top row
        self.obstacle_mask[:, 0] = True  # left column
        self.obstacle_mask[:, -1] = True  # right column

        # Ray state (global)
        self.current_ray = []
        self.current_hit = None

        # Animation state
        self.angle_idx = 0
        self.angles = list(range(0, 360, 1))

        # Set up figure and grid
        self.fig, self.ax = plt.subplots(figsize=(6, 6))

        # Pre-create imshow object (for fast update)
        self.img = self.ax.imshow(np.zeros((self.grid_size, self.grid_size), dtype=np.int8), cmap='gray_r',
                                  origin='lower',
                                  extent=(0, self.grid_size, 0, self.grid_size), vmin=-1, vmax=100)
        self.ax.set_xlim(0, self.grid_size)
        self.ax.set_ylim(0, self.grid_size)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_aspect('equal')
        self.ax.grid(True, which='both', color='lightgray', linewidth=0.5)

        # Rectangle placeholders for ray and hit visualization
        self.ray_rects = []
        self.hit_rect = None
        self.lidar_rect = plt.Rectangle(self.center, 1, 1, facecolor='red')
        self.ax.add_patch(self.lidar_rect)

    # ray_cast laser
    def ray_casting(self, x0, y0, x1, y1):  # bresenham line algorithm. based on pseudocode available at:
        # https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        if dx > dy:
            err = dx // 2
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy // 2
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

    # Animation frame update
    def update(self, _):
        for r in self.ray_rects:
            r.remove()
        self.ray_rects.clear()
        if self.hit_rect:
            self.hit_rect.remove()
            self.hit_rect = None

        self.current_ray = []
        self.current_hit = None

        angle = radians(self.angles[self.angle_idx])
        cx, cy = self.center  # center of lidar
        x1 = int(cx + self.radius * cos(angle))  # polar to cartesian conversion
        y1 = int(cy + self.radius * sin(angle))

        full_ray = self.ray_casting(cx, cy, x1, y1)

        for x, y in full_ray:
            if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):  # this skips any cells that are out of bounds
                break

            # --------------------------- TBD ------------------------
            # this is the log odds updates for the occupied grid.

            if self.obstacle_mask[y, x]:
                lt_1 = ...
                lt = ...  
                self.log_odds[y, x] = np.clip(lt, self.log_min, self.log_max)  # this to keep it within a certain range
                self.current_hit = (x, y)
                break

            # this is the log odds updates for the free grid.

            else:
                lt_1 = ...
                lt = ...
                self.log_odds[y, x] = np.clip(lt, self.log_min, self.log_max)
                self.current_ray.append((x, y))
            # --------------------------- TBD- END ------------------------
        self.angle_idx = (self.angle_idx + 1) % len(self.angles)

        # --------------------------- TBD ------------------------
        # Convert log-odds to occupancy grid values: unknown = -1, else scale probability [0–100]
        exp_log_odds = ...
        prob = ...  # by this time you have log odds, convert to probability.
        display = ...  # convert the range accordingly. this is also done in ROS2. also it should be int8 type
        display[np.isclose(np.exp(self.log_odds), np.exp(self.l0))] = ... # value for unknown
        # --------------------------- TBD - END ------------------------

        self.img.set_data(display)

        for x, y in self.current_ray:
            r = plt.Rectangle((x, y), 1, 1, facecolor='red', alpha=0.5)
            self.ax.add_patch(r)
            self.ray_rects.append(r)

        if self.current_hit:
            xh, yh = self.current_hit
            if 0 <= xh < self.grid_size and 0 <= yh < self.grid_size:
                self.hit_rect = plt.Rectangle((xh, yh), 1, 1, facecolor='yellow', alpha=0.8)
                self.ax.add_patch(self.hit_rect)

        return [self.img] + self.ray_rects + ([self.hit_rect] if self.hit_rect else [])

    def run(self):
        ani = animation.FuncAnimation(self.fig, self.update, interval=1, blit=True, frames=len(self.angles))
        plt.show()


if __name__ == '__main__':
    sim = OccupancyGridMappingSim()
    sim.run()
