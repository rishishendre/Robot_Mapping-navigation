import numpy as np

class PurePursuitController:
    def __init__(self, lookahead_dist=0.5, max_linear_vel=0.2, max_angular_vel=1.0):
        self.lookahead_dist = lookahead_dist
        self.max_linear = max_linear_vel
        self.max_angular = max_angular_vel

    def get_lookahead_point(self, path, robot_x, robot_y):
        """
        Walk along path from current position,
        return the first point that is lookahead_dist away.
        """
        for i in range(len(path) - 1, -1, -1):
            px, py = path[i]
            dist = np.sqrt((px - robot_x)**2 + (py - robot_y)**2)
            if dist <= self.lookahead_dist:
                # return the NEXT point after this one (or last)
                if i + 1 < len(path):
                    return path[i + 1]
                return path[i]
        return path[-1]  # fallback: last point

    def compute_velocity(self, robot_x, robot_y, robot_yaw, path):
        """
        Returns (linear_vel, angular_vel)
        """
        if not path:
            return 0.0, 0.0

        goal_x, goal_y = path[-1]
        dist_to_goal = np.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        if dist_to_goal < 0.1:  # within 10cm
            return 0.0, 0.0  # stop

        # Find lookahead point
        lx, ly = self.get_lookahead_point(path, robot_x, robot_y)

        # Transform lookahead point to robot frame
        dx = lx - robot_x
        dy = ly - robot_y

        # Angle to lookahead in world frame
        angle_to_lookahead = np.arctan2(dy, dx)

        # How much do we need to turn?
        alpha = angle_to_lookahead - robot_yaw

        # Normalize to [-pi, pi]
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        # Pure pursuit curvature formula: κ = 2*sin(α) / L
        L = self.lookahead_dist
        curvature = 2.0 * np.sin(alpha) / L

        # Scale linear vel down when turning sharply
        linear_vel = self.max_linear * (1.0 - 0.5 * abs(alpha) / np.pi)
        angular_vel = linear_vel * curvature

        # Clamp
        linear_vel  = np.clip(linear_vel,  0.0, self.max_linear)
        angular_vel = np.clip(angular_vel, -self.max_angular, self.max_angular)

        return linear_vel, angular_vel    
