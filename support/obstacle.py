import math


class Obstacle:
    """
    Class representing a rectangular obstacle. You may add to this class if you wish, but you should not modify the
    existing functions or variable names.

    COMP3702 2019 Assignment 2 Support Code

    Last updated by njc 24/08/19
    """

    MAX_DIST = 0.2
    MAX_DEPTH = 8

    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        assert x1 < x2, "For a valid obstacle, mush have x1 < x2"
        assert y1 < y2, "For a valid obstacle, mush have y1 < y2"

        self.corners = [(x1, y1), (x1, y2), (x2, y2), (x2, y1)]
        self.edges = [(self.corners[i], self.corners[(i + 1) % 4]) for i in range(4)]

        self.hbv_top_level = [HBVNode(*e) for e in self.edges]

    def dist_to_point(self, x, y):
        # compute a lower bound on the distance between the point and the obstacle
        hbv_level = self.hbv_top_level
        depth = 0
        while True:
            min_dist = 2    # dist can be at most sqrt(2)
            min_idx = -1
            for i in range(len(hbv_level)):
                dist = hbv_level[i].get_dist((x, y))
                if dist < min_dist:
                    min_dist = dist
                    min_idx = i

            if min_dist > self.MAX_DIST:
                return self.MAX_DIST

            # go down one level and choose closest
            depth += 1
            if depth > self.MAX_DEPTH:
                break
            hbv_level = hbv_level[min_idx].get_children()

        return max(min_dist, 0)

    def move_is_possible(self, x, y, m):
        # useful?
        hbv_level = self.hbv_top_level
        depth = 0
        while True:
            min_dist = 2  # dist can be at most sqrt(2)
            min_idx = -1
            for i in range(len(hbv_level)):
                dist = hbv_level[i].get_dist((x, y))
                if dist < min_dist:
                    min_dist = dist
                    min_idx = i

            if min_dist > m:
                return True

            # go down one level and choose closest
            depth += 1
            if depth > self.MAX_DEPTH:
                break
            hbv_level = hbv_level[min_idx].get_children()

        return False


class HBVNode:
    """
    Class representing a hierarchical bounding volume tree node.
    """
    def __init__(self, p, q, parent=None):
        x1, y1 = p
        x2, y2 = q
        if x1 == x2:
            self.centre = (x1, (y1 + y2) / 2)
            self.radius = abs(y2 - y1) / 2

        elif y1 == y2:
            self.centre = ((x1 + x2) / 2, y1)
            self.radius = abs(x2 - x1) / 2
        else:
            raise Exception("HBVNode Error: Coordinates which do not form an axis aligned line are not supported")

        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.parent = parent
        self.children = None

    def get_dist(self, p):
        x, y = p
        return math.sqrt((self.centre[0] - x)**2 + (self.centre[1] - y)**2) - self.radius

    def get_children(self):
        if self.children is None:
            self.children = [HBVNode((self.x1, self.y1), self.centre, self),
                             HBVNode(self.centre, (self.x2, self.y2), self)]
        return self.children


