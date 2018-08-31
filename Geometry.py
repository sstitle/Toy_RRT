import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
import math

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, v):
        return Vector(self.x + v.x, self.y + v.y)

    def __sub__(self, v):
        return Vector(self.x - v.x, self.y - v.y)

    def __rmul__(self, c):
        return Vector(c*self.x, c*self.y)

    def __str__(self):
         return "({},{})".format(self.x, self.y)


def SegSegIntersect(a1, a2, b1, b2):
    b = a2 - a1
    d = b2 - b1

    dot_perp = b.x*d.y - b.y*d.x;
    if dot_perp == 0:
        return False

    c = b1 - a1
    t = (c.x*d.y - c.y*d.x) / dot_perp
    if t < 0.0 or t > 1.0:
        return False

    u = (c.x*b.y - c.y*b.x) / dot_perp;
    if u < 0.0 or u > 1.0:
        return False
    return True


def RectSegIntersect(rect, segment):
    center, size = rect
    s1, s2 = segment
    corner_1 = Vector(center.x - 0.5*size.x, center.y - 0.5*size.y)
    corner_2 = Vector(center.x + 0.5*size.x, center.y - 0.5*size.y)
    corner_3 = Vector(center.x - 0.5*size.x, center.y + 0.5*size.y)
    corner_4 = Vector(center.x + 0.5*size.x, center.y + 0.5*size.y)

    return SegSegIntersect(corner_1, corner_2, s1, s2) or \
           SegSegIntersect(corner_2, corner_4, s1, s2) or \
           SegSegIntersect(corner_4, corner_3, s1, s2) or \
           SegSegIntersect(corner_3, corner_1, s1, s2)


def RectPointIntersect(rect, point):
    center, size = rect
    corner_1 = Vector(center.x - 0.5*size.x, center.y - 0.5*size.y)
    corner_4 = Vector(center.x + 0.5*size.x, center.y + 0.5*size.y)
    if point.x < corner_1.x or point.x > corner_4.x:
        return False
    if point.y < corner_1.y or point.y > corner_4.y:
        return False
    return True


def NodeCollision(node, obstacles):
    for rect in obstacles:
        if RectPointIntersect(rect, node):
            return True
    return False


def EdgeCollision(node_a, node_b, obstacles):
    for rect in obstacles:
        if RectSegIntersect(rect, (node_a, node_b)):
            return True
    return False


def Distance(a, b):
    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)


def Clamp(val, min_val, max_val):
    return min(max_val, max(min_val, val))


def InterpolateLinear(u, a, b):
    return a + u*(b - a);


################################################################################
## Plotting
################################################################################

def PlotSeg(node, ngbr, style='k-'):
    plt.plot([node.x, ngbr.x], [node.y, ngbr.y], style)


def PlotRects(rects, ax):
    for rect in rects:
        PlotRect(rect, ax)


def PlotRect(rect, ax):
    center, size = rect
    corner = (center.x - 0.5*size.x, center.y - 0.5*size.y)
    patches = [mpatches.Rectangle(corner, size.x, size.y, ec="none")]
    collection = PatchCollection(patches, cmap=plt.cm.hsv)
    ax.add_collection(collection)


def PlotPath(path, style='g-'):
    x = [node.x for node in path]
    y = [node.y for node in path]
    plt.plot(x, y, style, linewidth=3)
