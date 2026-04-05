"""
navigation.py — Global path planning: obstacle inflation, visibility graph, Dijkstra.

Functions / Classes:
    checkbBlackOnLine  — pixel-by-pixel line-of-sight check between two points
    visibility_map     — builds a connection table (distance or inf) for all edge pairs
    rawToBinary        — inflates obstacles by robot radius and renders a B&W map
    getRightedges      — flattens obstacle polygon corners + start/goal into an edge array
    Graph              — graph class built from the connection table (adjacency dict)
    djikstra           — Dijkstra shortest-path over the visibility graph
    path_construction  — end-to-end pipeline: inflate → visibility → Dijkstra → waypoints
"""

import numpy as np
import cv2
import math
import copy
import sys
import matplotlib.pyplot as plt

DEBUG = False  # set True to pop visualisation windows during development

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

IMAGE_WIDTH  = 1920
IMAGE_HEIGHT = 1080
IMAGE_ORIGIN = 0

RED_COLOR_AXIS   = 0
GREEN_COLOR_AXIS = 1
BLUE_COLOR_AXIS  = 2
MAX_COLOR_VALUE  = 255

BLUE_COLOR           = [0, 0, 255]
BLUE_LINE_THICKNESS  = 2

THYMIO_SIZE = 140

COUNTER_START     = 0
COUNTER_INCREMENT = 1

POLYGON_COLOR = (0, 0, 0)

VISBILITY_MARGING = 12

NUMBER_OF_ADDED_ELEMENT_TO_EDGES = 2
NUMBER_OF_DIMENSIONS             = 2

X_AXIS = 0
Y_AXIS = 1

GOAL_POS   = 1
THYMIO_POS = 0
COLOR_RED      = [255, 0, 0]
LINE_THICKNESS = 5

INITIAL_SHORTEST_PATH = 0

# ---------------------------------------------------------------------------
# Visibility helpers
# ---------------------------------------------------------------------------

def checkbBlackOnLine(img, p1, p2):
    """Return True if any pixel along the line segment p1→p2 is non-white (i.e. blocked).

    :param img: BGR image (white = free, black = obstacle)
    :param p1: start point as int numpy array [x, y]
    :param p2: end point as int numpy array [x, y]
    :returns: True if the segment crosses an obstacle or leaves the image bounds
    """
    black = False
    if (p1[X_AXIS] >= IMAGE_WIDTH  or p1[X_AXIS] <= IMAGE_ORIGIN or
        p1[Y_AXIS] >= IMAGE_HEIGHT or p1[Y_AXIS] <= IMAGE_ORIGIN or
        p2[X_AXIS] >= IMAGE_WIDTH  or p2[X_AXIS] <= IMAGE_ORIGIN or
        p2[Y_AXIS] >= IMAGE_HEIGHT or p2[Y_AXIS] <= IMAGE_ORIGIN):
        return True

    v    = (p2 - p1) / np.linalg.norm((p2 - p1))
    a    = p1
    iter = np.linalg.norm((p2 - p1)) / np.linalg.norm(v)
    for i in range(int(iter) - 1):
        a = a + v
        R = img[int(a[Y_AXIS]), int(a[X_AXIS]), RED_COLOR_AXIS]
        G = img[int(a[Y_AXIS]), int(a[X_AXIS]), GREEN_COLOR_AXIS]
        B = img[int(a[Y_AXIS]), int(a[X_AXIS]), BLUE_COLOR_AXIS]
        if R != MAX_COLOR_VALUE and G != MAX_COLOR_VALUE and B != MAX_COLOR_VALUE:
            black = True
            break
    return black


def visibility_map(img, edges):
    """Build an N×N connection table of Euclidean distances (or inf if blocked) between edges.

    :param img: white-background obstacle map (BGR); visible edges are drawn on it for debug
    :param edges: (N, 2) int array of [x, y] graph nodes (start, goal, obstacle corners)
    :returns: (N, N) numpy array — entry [i,j] is distance if visible, inf if not, 0 on diagonal
    """
    img2     = img.copy()
    nbEdges  = np.shape(edges)[X_AXIS]
    conTab   = np.zeros((nbEdges, nbEdges))

    for i in range(np.shape(edges)[X_AXIS]):
        for j in range(np.shape(edges)[X_AXIS]):
            if i != j:
                if checkbBlackOnLine(img2, edges[i], edges[j]) == False:
                    cv2.line(img, edges[i], edges[j], BLUE_COLOR, thickness=BLUE_LINE_THICKNESS)
                    conTab[i, j] = int(np.linalg.norm(edges[i] - edges[j]))
                else:
                    conTab[i, j] = "inf"
            else:
                conTab[i, j] = 0
    return conTab


def rawToBinary(img, obst):
    """Inflate each obstacle polygon by THYMIO_SIZE pixels and render a white B&W map.

    :param img: original camera image (used only for shape/dtype)
    :param obst: list of obstacle polygon corner arrays (from vision.obstacle_detection)
    :returns: (binary_image, inflated_obst) where inflated_obst has corners pushed outward
    """
    image   = img.copy()
    image[:, :, :] = MAX_COLOR_VALUE
    further = THYMIO_SIZE
    cnt1    = COUNTER_START
    cnt2    = COUNTER_START

    obst2 = copy.deepcopy(obst)
    obst3 = copy.deepcopy(obst)
    for i in obst:
        center = np.sum(i, axis=0) / np.shape(i)[X_AXIS]
        cnt2   = COUNTER_START
        for j in i:
            k = copy.deepcopy(j)
            p = (j - center) / (np.linalg.norm(j - center))
            j = j + further * p
            j = j.astype(int)
            obst2[cnt1][cnt2] = j

            k = k + (further - VISBILITY_MARGING) * p
            k = k.astype(int)
            obst3[cnt1][cnt2] = k
            cnt2 += COUNTER_INCREMENT
        pts   = np.array(obst3[cnt1])
        image = cv2.fillPoly(image, [pts], color=POLYGON_COLOR)
        cnt1 += COUNTER_INCREMENT
    return image, obst2


def getRightedges(poly_obst, cent_thym, cent_goal):
    """Flatten obstacle corners into a single (N, 2) edge array with start and goal prepended.

    :param poly_obst: list of obstacle polygon arrays (may be empty)
    :param cent_thym: [x, y] Thymio center — placed at index 0 (THYMIO_POS)
    :param cent_goal: [x, y] goal center — placed at index 1 (GOAL_POS)
    :returns: (N, 2) int array of all graph nodes
    """
    if poly_obst == []:
        size = 0
    else:
        size = np.shape(poly_obst)[X_AXIS] * np.shape(poly_obst)[Y_AXIS]
    edges       = np.zeros((size + NUMBER_OF_ADDED_ELEMENT_TO_EDGES, NUMBER_OF_DIMENSIONS))
    c           = NUMBER_OF_ADDED_ELEMENT_TO_EDGES
    edges[X_AXIS] = cent_thym
    edges[Y_AXIS] = cent_goal
    for i in poly_obst:
        for j in i:
            edges[c, X_AXIS] = np.int64(j[0][X_AXIS])
            edges[c, Y_AXIS] = np.int64(j[0][Y_AXIS])
            c += COUNTER_INCREMENT
    edges = edges.astype(int)
    return edges


# ---------------------------------------------------------------------------
# Graph
# ---------------------------------------------------------------------------

class Graph:
    """Adjacency-list graph built from the visibility connection table.

    :param tab: (N, N) connection table produced by visibility_map
    """

    def __init__(self, tab):
        self.nodes = 0
        self.graph = self.construct_graph(tab)

    def construct_graph(self, tab):
        """Convert the connection table into a dict mapping node → list of reachable nodes."""
        graphN  = {}
        nbNodes = np.shape(tab)[X_AXIS]
        for i in range(nbNodes):
            for j in range(nbNodes):
                if tab[i, j] != float('inf') and tab[i, j] != 0:
                    if i in graphN:
                        graphN[i].append(j)
                    else:
                        graphN[i] = [j]
        return graphN

    def get_nodes(self):
        """Return the set of node indices present in the graph."""
        return self.graph.keys()

    def get_graph(self):
        """Return the adjacency dict."""
        return self.graph


# ---------------------------------------------------------------------------
# Dijkstra
# ---------------------------------------------------------------------------

def djikstra(graph, conTab, start_node, end_node):
    """Run Dijkstra on the visibility graph and return the node sequence from start to end.

    :param graph: adjacency dict from Graph.get_graph()
    :param conTab: (N, N) connection table with edge weights
    :param start_node: index of the start node (0 = Thymio)
    :param end_node: index of the goal node (1 = goal)
    :returns: list of node indices from start_node to end_node (reversed internally then corrected)
    """
    unvisited_nodes = list(graph)
    shortest_path   = {}
    previous_nodes  = {}
    max_dist        = sys.maxsize
    for node in unvisited_nodes:
        shortest_path[node] = max_dist
    shortest_path[start_node] = INITIAL_SHORTEST_PATH

    while unvisited_nodes:
        current_min_node = None
        for node in unvisited_nodes:
            if current_min_node is None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node
        neighbors = graph.get(current_min_node)
        for neighbor in neighbors:
            temp_value = shortest_path[current_min_node] + conTab[current_min_node, neighbor]
            if temp_value < shortest_path[neighbor]:
                shortest_path[neighbor] = temp_value
                previous_nodes[neighbor] = current_min_node
        unvisited_nodes.remove(current_min_node)

    path = []
    node = end_node
    while node != start_node:
        path.append(node)
        node = previous_nodes[node]
    path.append(start_node)
    return path


# ---------------------------------------------------------------------------
# End-to-end path construction
# ---------------------------------------------------------------------------

def path_construction(image, poly_obst, cent_thym, cent_goal):
    """Compute the shortest collision-free waypoint path from Thymio to goal.

    Inflates obstacles by robot radius, builds a visibility graph, runs Dijkstra,
    and returns the ordered list of [x, y] waypoints to follow.

    :param image: raw camera frame (BGR)
    :param poly_obst: obstacle polygon list from vision.obstacle_detection
    :param cent_thym: [x, y] Thymio center
    :param cent_goal: [x, y] goal center
    :returns: list of [x, y] waypoints from Thymio position to goal
    """
    img, poly_obst = rawToBinary(image, poly_obst)
    edges          = getRightedges(poly_obst, cent_thym, cent_goal)
    conTab         = visibility_map(img, edges)
    gr             = Graph(conTab).get_graph()
    path           = djikstra(gr, conTab, THYMIO_POS, GOAL_POS)

    path_pos = []
    for i in reversed(path):
        path_pos.append(edges[i])

    prev = path_pos[0]
    for i in path_pos:
        cv2.line(img, prev, i, COLOR_RED, thickness=LINE_THICKNESS)
        prev = i
    if DEBUG:
        plt.imshow(img)

    return path_pos
