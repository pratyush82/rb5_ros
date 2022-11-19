import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
plt.rcParams["font.family"] = "Times"

class Pose2D:
    def __init__(self, x, y, theta=0) -> None:
        self.x = x
        self.y = y
        self.theta = theta

    def __sub__(self, other):
        return np.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

    def __str__(self):
        return f"({self.x:.2f},{self.y:.2f},{self.theta:.2f})"

class Planner:
    SAFETY_THRESHOLD = 0.15

    def __init__(self, start: Pose2D, stop: Pose2D, obstacles=[]):
        self.start = start
        self.stop = stop
        self.current: Pose2D = start
        self.delta = 0.01
        self.compute_line()
        self.obstacles = obstacles
        self.motion = 'normal'
        self.waypoints = [start]

    def add_obstacle(self, center: Pose2D, width: float, height: float):
        self.obstacles.append(
            [center, width+Planner.SAFETY_THRESHOLD, height+Planner.SAFETY_THRESHOLD])

    def is_free(self, x, y, tol=1e-2, go_around=True) -> bool:
        for ob in self.obstacles:
            center, width, height = ob
            top = center.y + height/2
            bottom = center.y - height/2
            left = center.x - width/2
            right = center.x + width/2
            if y-bottom >= tol and y-top <= tol and x-left >= tol and x-right <= tol:
                print(f"{(x,y)} is blocked by obstacle {str(ob[0]),ob[1:]}")
                if go_around:
                    self.go_around_obstacle(ob, tol=tol)
                return False
        return True

    def go_around_obstacle(self, obstacle, tol):
        obstacle_path = []
        done: bool = False
        obstacle_path_length = 0
        obstacle_waypoints = []
        center, width, height = obstacle
        top = center.y + height/2
        bottom = center.y - height/2
        left = center.x - width/2
        right = center.x + width/2
        previous_pose = Pose2D(
            self.current.x, self.current.y, self.current.theta)

        # Counter-clockwise path
        # Right
        while self.current.x-right <= tol and not done:
            x_new = self.current.x + self.delta
            self.current = Pose2D(x_new, self.current.y, 0)
            obstacle_path.append(self.current)
            obstacle_path_length += 1
            if abs(self.current.y - self.m*self.current.x - self.c) <= tol:
                done = True

        # Up
        if not done:
            obstacle_waypoints.append(self.current)
        while self.current.y-top <= tol and not done:
            y_new = self.current.y + self.delta
            self.current = Pose2D(self.current.x, y_new, 0)
            obstacle_path.append(self.current)
            obstacle_path_length += 1
            if abs(self.current.y - self.m*self.current.x - self.c) <= tol:
                done = True

        # Left
        if not done:
            obstacle_waypoints.append(self.current)
        while self.current.x-left >= tol and not done:
            x_new = self.current.x - self.delta
            self.current = Pose2D(x_new, self.current.y, 0)
            obstacle_path.append(self.current)
            obstacle_path_length += 1
            if abs(self.current.y - self.m*self.current.x - self.c) <= tol:
                done = True

        # Down
        if not done:
            obstacle_waypoints.append(self.current)
        while self.current.y-bottom >= tol and not done:
            y_new = self.current.y - self.delta
            self.current = Pose2D(self.current.x, y_new, 0)
            obstacle_path.append(self.current)
            obstacle_path_length += 1
            if abs(self.current.y - self.m*self.current.x - self.c) <= tol:
                done = True

        print("Obstacle path length", obstacle_path_length)
        print("Half perimeter", (width+height)/self.delta)
        # This path is shorter
        if done and obstacle_path_length <= (width+height)/self.delta:
            print("Did Counter-clockwise rotation around obstacle and reached", self.current)
            self.waypoints.extend(obstacle_waypoints)
            self.path.extend(obstacle_path)
            return

        # Clockwise rotation
        print("Resetting current from", self.current, "to", previous_pose)
        done = False
        self.current = previous_pose
        obstacle_waypoints = []
        obstacle_path = []
        obstacle_path_length = 0

        # Left
        while self.current.x-left >= tol and not done:
            x_new = self.current.x - self.delta
            self.current = Pose2D(x_new, self.current.y, 0)
            obstacle_path.append(self.current)
            obstacle_path_length += 1
            if abs(self.current.y - self.m*self.current.x - self.c) <= tol:
                done = True

        # Up
        if not done:
            obstacle_waypoints.append(self.current)
        while self.current.y-top <= tol and not done:
            y_new = self.current.y + self.delta
            self.current = Pose2D(self.current.x, y_new, 0)
            obstacle_path.append(self.current)
            obstacle_path_length += 1
            if abs(self.current.y - self.m*self.current.x - self.c) <= tol:
                done = True

        # Right
        obstacle_waypoints.append(self.current)
        while self.current.x-right <= tol and not done:
            x_new = self.current.x + self.delta
            self.current = Pose2D(x_new, self.current.y, 0)
            # print("Right:",self.current)
            obstacle_path.append(self.current)
            obstacle_path_length += 1
            if abs(self.current.y - self.m*self.current.x - self.c) <= tol:
                # obstacle_waypoints.append(self.current)
                done = True

        # Down
        if not done:
            obstacle_waypoints.append(self.current)
        while self.current.y-bottom >= tol and not done:
            y_new = self.current.y - self.delta
            self.current = Pose2D(self.current.x, y_new, 0)
            obstacle_path.append(self.current)
            obstacle_path_length += 1
            if abs(self.current.y - self.m*self.current.x - self.c) <= tol:
                done = True

        print("Did clockwise rotation around obstacle and reached", self.current)
        self.waypoints.extend(obstacle_waypoints)
        self.path.extend(obstacle_path)

    def compute_line(self):
        x1, y1, x2, y2 = self.start.x, self.start.y, self.stop.x, self.stop.y
        self.m: float = (y2-y1)/(x2-x1)
        self.c = y1 - self.m * x1

    def check_line_free(self, x1, y1, x2, y2):
        m: float = (y2-y1)/(x2-x1)
        c = y1 - self.m * x1
        while x2 - x1 > self.delta:
            x1 = x1 + self.delta
            y1 = m*x1 + c
            if not self.is_free(x1, y1, tol=self.delta, go_around=False):
                return False
        return True

    def compute_path(self):
        self.path = [self.current]
        while self.current - self.stop > self.delta:
            x_new = self.current.x + self.delta
            y_new = self.m*x_new + self.c
            if self.is_free(x_new, y_new, self.delta):
                self.current = Pose2D(x_new, y_new, np.arctan(self.m))
            self.path.append(self.current)
        self.waypoints.append(self.stop)

    def shorten_path(self):
        while True:
            current_length = len(self.waypoints)
            i = 1
            while i+1 < len(self.waypoints):
                prev = self.waypoints[i-1]
                next = self.waypoints[i+1]
                if self.check_line_free(prev.x, prev.y, next.x, next.y):
                    self.waypoints.pop(i)
                i += 1
            if current_length == len(self.waypoints):
                break

    def plot_path(self):
        fig = plt.figure(dpi=150, figsize=(4, 4))
        plt.title("Bug 2 Path")
        for wp in self.path:
            plt.plot(wp.x, wp.y, 'r.', markersize=1)
        for ob in self.obstacles:
            center, width, height = ob[0], ob[1] - \
                Planner.SAFETY_THRESHOLD, ob[2]-Planner.SAFETY_THRESHOLD
            fig.gca().add_patch(Rectangle((center.x-width/2, center.y-height/2), width, height,))
        plt.show()

    def plot_waypoints(self):
        fig = plt.figure(dpi=150, figsize=(4, 4))
        plt.ylim([-0.1, 1.1])
        plt.xlim([-0.15, 1.15])
        plt.title("Shortest path")
        plt.plot([self.start.x, self.stop.x], [
                 self.start.y, self.stop.y], 'r--')
        plt.text(self.waypoints[0].x-0.12, self.waypoints[0].y-0.06,
                 f"({self.waypoints[0].x:.2f},{self.waypoints[0].y:.2f})")
        for i in range(1, len(self.waypoints)):
            plt.plot([self.waypoints[i-1].x, self.waypoints[i].x],
                     [self.waypoints[i-1].y, self.waypoints[i].y], '-g.')
            plt.text(self.waypoints[i].x-0.12, self.waypoints[i].y+0.02,
                     f"({self.waypoints[i].x:.2f},{self.waypoints[i].y:.2f})")
        for ob in self.obstacles:
            center, width, height = ob[0], ob[1] - \
                Planner.SAFETY_THRESHOLD, ob[2]-Planner.SAFETY_THRESHOLD
            fig.gca().add_patch(Rectangle((center.x-width/2, center.y-height/2), width, height,))
        plt.show()

# Main function
start = Pose2D(0, 0, 0)
stop = Pose2D(1, 1, 0)
planner = Planner(start, stop)
planner.add_obstacle(Pose2D(0.5, 0.5), 1/3, 1/3)
planner.compute_path()
planner.plot_path()
planner.shorten_path()
planner.plot_waypoints()
