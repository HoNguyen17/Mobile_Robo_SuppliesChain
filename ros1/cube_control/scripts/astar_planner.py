#!/usr/bin/env python3
import heapq
import math
import time
from collections import deque

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

SQRT2 = math.sqrt(2.0)


class AStarPlanner:
    def __init__(self):
        self.snap_distance = rospy.get_param('~snap_distance', 6.0)  # same idea as Snap Distance in C#
        self.print_map = rospy.get_param('~print_map', True)         # draw the path as text after planning

        self.data = None
        self.width = 0
        self.height = 0
        self.res = 0.5
        self.ox = 0.0
        self.oy = 0.0
        self.pose = None

        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
        rospy.Subscriber('/map', OccupancyGrid, self.on_map)
        rospy.Subscriber('/cube/pose', Pose, self.on_pose)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.on_goal)
        rospy.loginfo('astar_planner ready: send a goal to /move_base_simple/goal')

    # ---------------- callbacks ----------------
    def on_map(self, msg):
        self.data = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y

    def on_pose(self, msg):
        self.pose = msg

    def on_goal(self, msg):
        if self.data is None:
            rospy.logwarn('No /map received yet.')
            return
        if self.pose is None:
            rospy.logwarn('No /cube/pose received yet.')
            return

        t0 = time.time()
        sx, sy = self.pose.position.x, self.pose.position.y
        gx, gy = msg.pose.position.x, msg.pose.position.y

        start = self.nearest_free(sx, sy, None)
        if start < 0:
            rospy.logwarn('No free cell near the robot.')
            return

        reachable = self.flood_fill(start)
        goal = self.nearest_free(gx, gy, reachable)
        if goal < 0:
            rospy.logwarn('No reachable free cell within %.1f m of the goal.', self.snap_distance)
            return

        cells = self.astar(start, goal)
        if cells is None:
            rospy.logwarn('A* found no route.')
            return

        points = [self.cell_center(c) for c in cells]
        # Use the exact positions at both ends when they were not snapped.
        if start == self.cell_index(sx, sy):
            points[0] = (sx, sy)
        if goal == self.cell_index(gx, gy):
            points[-1] = (gx, gy)
        else:
            rospy.logwarn('Goal (%.2f, %.2f) is blocked or unreachable: snapped to (%.2f, %.2f).',
                          gx, gy, points[-1][0], points[-1][1])

        corners = self.smooth(points)
        self.publish_path(corners)

        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(corners, corners[1:]))
        rospy.loginfo('Planned: %d cells -> %d corners, %.1f m, %.0f ms',
                      len(cells), len(corners), length, (time.time() - t0) * 1000.0)
        for k, c in enumerate(corners):
            rospy.loginfo('  corner %d: x=%.2f y=%.2f', k, c[0], c[1])
        if self.print_map:
            self.print_ascii(corners)

    # ---------------- grid helpers ----------------
    def cell_index(self, x, y):
        i = int(math.floor((x - self.ox) / self.res))
        j = int(math.floor((y - self.oy) / self.res))
        if i < 0 or j < 0 or i >= self.width or j >= self.height:
            return -1
        return j * self.width + i

    def cell_center(self, idx):
        i = idx % self.width
        j = idx // self.width
        return (self.ox + (i + 0.5) * self.res, self.oy + (j + 0.5) * self.res)

    def blocked(self, idx):
        return self.data[idx] != 0

    def nearest_free(self, x, y, mask):
        """Nearest free cell within snap_distance. If mask is given, the cell must also be in it."""
        w, h = self.width, self.height
        cx = min(max(int(math.floor((x - self.ox) / self.res)), 0), w - 1)
        cz = min(max(int(math.floor((y - self.oy) / self.res)), 0), h - 1)
        max_r = int(math.ceil(self.snap_distance / self.res))
        best, best_sqr = -1, float('inf')
        for dj in range(-max_r, max_r + 1):
            for di in range(-max_r, max_r + 1):
                i, j = cx + di, cz + dj
                if i < 0 or j < 0 or i >= w or j >= h:
                    continue
                idx = j * w + i
                if self.blocked(idx):
                    continue
                if mask is not None and not mask[idx]:
                    continue
                px, py = self.cell_center(idx)
                sqr = (px - x) ** 2 + (py - y) ** 2
                if sqr < best_sqr:
                    best_sqr, best = sqr, idx
        return best

    def flood_fill(self, start):
        """Every free cell reachable from start (4-connected)."""
        w, h = self.width, self.height
        visited = [False] * (w * h)
        visited[start] = True
        queue = deque([start])
        while queue:
            cell = queue.popleft()
            cx, cz = cell % w, cell // w
            for nx, nz in ((cx + 1, cz), (cx - 1, cz), (cx, cz + 1), (cx, cz - 1)):
                if nx < 0 or nz < 0 or nx >= w or nz >= h:
                    continue
                ni = nz * w + nx
                if visited[ni] or self.blocked(ni):
                    continue
                visited[ni] = True
                queue.append(ni)
        return visited

    # ---------------- A* ----------------
    def heuristic(self, a, b):
        w = self.width
        dx = abs(a % w - b % w)
        dz = abs(a // w - b // w)
        return (dx + dz) + (SQRT2 - 2.0) * min(dx, dz)   # octile distance

    def astar(self, start, goal):
        w, h = self.width, self.height
        n = w * h
        g = [float('inf')] * n
        parent = [-1] * n
        closed = [False] * n
        g[start] = 0.0
        open_heap = [(self.heuristic(start, goal), start)]   # (f, cell), smallest f pops first

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if closed[current]:
                continue                                      # stale duplicate entry
            if current == goal:
                path = []
                while current != -1:
                    path.append(current)
                    current = parent[current]
                path.reverse()
                return path

            closed[current] = True
            cx, cz = current % w, current // w
            for dz in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dz == 0:
                        continue
                    nx, nz = cx + dx, cz + dz
                    if nx < 0 or nz < 0 or nx >= w or nz >= h:
                        continue
                    ni = nz * w + nx
                    if self.blocked(ni) or closed[ni]:
                        continue
                    diagonal = dx != 0 and dz != 0
                    # No squeezing diagonally between two blocked cells.
                    if diagonal and (self.blocked(cz * w + nx) or self.blocked(nz * w + cx)):
                        continue
                    new_g = g[current] + (SQRT2 if diagonal else 1.0)
                    if new_g < g[ni]:
                        g[ni] = new_g
                        parent[ni] = current
                        heapq.heappush(open_heap, (new_g + self.heuristic(ni, goal), ni))
        return None

    # ---------------- smoothing ----------------
    def line_of_sight(self, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        steps = max(1, int(math.ceil(math.hypot(dx, dy) / (self.res * 0.5))))
        for s in range(steps + 1):
            t = s / float(steps)
            idx = self.cell_index(a[0] + dx * t, a[1] + dy * t)
            if idx < 0 or self.blocked(idx):
                return False
        return True

    def smooth(self, points):
        """Keep only the points needed so every straight segment stays inside free cells."""
        result = [points[0]]
        i = 0
        while i < len(points) - 1:
            j = len(points) - 1
            while j > i + 1 and not self.line_of_sight(points[i], points[j]):
                j -= 1
            result.append(points[j])
            i = j
        return result

    # ---------------- output ----------------
    def publish_path(self, corners):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        for k, (x, y) in enumerate(corners):
            if k < len(corners) - 1:
                yaw = math.atan2(corners[k + 1][1] - y, corners[k + 1][0] - x)
            else:
                yaw = math.atan2(y - corners[k - 1][1], x - corners[k - 1][0]) if k > 0 else 0.0
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        self.path_pub.publish(path)

    def print_ascii(self, points):
        """Same picture as map_viewer.py (top = +x, left = +y) with the path: S start, * path, G goal."""
        w, h = self.width, self.height
        if h > 100:
            return
        marks = {}
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            steps = max(1, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / (self.res * 0.5))))
            for s in range(steps + 1):
                t = s / float(steps)
                idx = self.cell_index(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
                if idx >= 0:
                    marks[idx] = '*'
        marks[self.cell_index(*points[0])] = 'S'
        marks[self.cell_index(*points[-1])] = 'G'
        for i in range(w - 1, -1, -1):
            line = ''
            for j in range(h - 1, -1, -1):
                idx = j * w + i
                if idx in marks:
                    line += marks[idx]
                elif self.blocked(idx):
                    line += '#'
                else:
                    line += '.'
            print(line)


if __name__ == '__main__':
    rospy.init_node('astar_planner')
    AStarPlanner()
    rospy.spin()
