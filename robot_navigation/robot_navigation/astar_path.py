import heapq
import numpy as np

class AStarPlanner:
    def __init__(self, occupancy_grid, resolution, origin):
        """
        occupancy_grid: 2D numpy array (0=free, 100=occupied, -1=unknown)
        resolution: meters per cell (e.g. 0.05)
        origin: (x, y) of cell (0,0) in world frame
        """
        self.grid = occupancy_grid
        self.resolution = resolution
        self.origin = origin  # (origin_x, origin_y)
        self.height, self.width = occupancy_grid.shape

    def world_to_cell(self, wx, wy):
        cx = int((wx - self.origin[0]) / self.resolution)
        cy = int((wy - self.origin[1]) / self.resolution)
        return (cx, cy)

    def cell_to_world(self, cx, cy):
        wx = cx * self.resolution + self.origin[0]
        wy = cy * self.resolution + self.origin[1]
        return (wx, wy)    
    
    def is_free(self, cx, cy):
        if cx < 0 or cy < 0 or cx >= self.width or cy >= self.height:
            return False
        val = self.grid[cy, cx]  # note: grid is [row=y, col=x]
        return val == 0  # only free cells

    def heuristic(self, a, b):
        # Euclidean distance heuristic
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def plan(self, start_world, goal_world):
        """
        start_world: (x, y) in meters
        goal_world:  (x, y) in meters
        returns: list of (x, y) world coords, or None if no path
        """
        start = self.world_to_cell(*start_world)
        goal  = self.world_to_cell(*goal_world)

        if not self.is_free(*start):
            print("Start cell is occupied!")
            return None
        if not self.is_free(*goal):
            print("Goal cell is occupied!")
            return None

        # Priority queue: (f_cost, cell)
        open_set = []  
        heapq.heappush(open_set, (0.0, start))

        came_from = {}           # cell → parent cell
        g_cost = {start: 0.0}   # cheapest known cost to reach cell

        # 8-connected neighbors (including diagonals)
        neighbors = [
            (1,0), (-1,0), (0,1), (0,-1),
            (1,1), (-1,1), (1,-1), (-1,-1)
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            # Goal reached
            if current == goal:
                return self._reconstruct_path(came_from, current)

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self.is_free(*neighbor):
                    continue

                # Diagonal moves cost more
                step_cost = 1.414 if dx != 0 and dy != 0 else 1.0
                new_g = g_cost[current] + step_cost

                if new_g < g_cost.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_cost[neighbor] = new_g
                    f = new_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return None  # no path found
    
    def _reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(self.cell_to_world(*current))
            current = came_from[current]
        path.append(self.cell_to_world(*current))
        path.reverse()
        return path  # list of (wx, wy)
