#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

MAX_COLS = 100  # shrink the picture if the map is wider than this


def main():
    rospy.init_node('map_viewer')
    grid = rospy.wait_for_message('/map', OccupancyGrid, timeout=10)
    try:
        pose = rospy.wait_for_message('/cube/pose', Pose, timeout=3)
    except rospy.ROSException:
        pose = None

    info = grid.info
    w, h, res = info.width, info.height, info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    step = max(1, int(math.ceil(h / float(MAX_COLS))))

    print('map: %d x %d cells, resolution %.2f m' % (w, h, res))
    print('origin (min x, min y): (%.2f, %.2f)' % (ox, oy))
    print('x range: %.2f .. %.2f   y range: %.2f .. %.2f'
          % (ox, ox + w * res, oy, oy + h * res))
    print('each character = %d x %d cells.  # = blocked, . = free, R = robot' % (step, step))
    print('top of picture = +x (forward), left of picture = +y (left)')

    def blocked(bi, bj):
        for j in range(bj * step, min(h, (bj + 1) * step)):
            for i in range(bi * step, min(w, (bi + 1) * step)):
                if grid.data[j * w + i] > 50:
                    return True
        return False

    robot = None
    if pose is not None:
        ri = int((pose.position.x - ox) / res)
        rj = int((pose.position.y - oy) / res)
        robot = (ri // step, rj // step)
        print('robot at x=%.2f y=%.2f -> cell (%d, %d)' % (pose.position.x, pose.position.y, ri, rj))

    n_bi = (w + step - 1) // step
    n_bj = (h + step - 1) // step
    for bi in range(n_bi - 1, -1, -1):        # x from high (top) to low (bottom)
        line = ''
        for bj in range(n_bj - 1, -1, -1):    # y from high (left) to low (right)
            if robot == (bi, bj):
                line += 'R'
            elif blocked(bi, bj):
                line += '#'
            else:
                line += '.'
        print(line)


if __name__ == '__main__':
    main()
