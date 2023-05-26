import astar_cpp as ac
from typing import Tuple
from numba import njit
import numpy as np

AStar = ac.AStar


@njit(cache=True)
def is_valid(x, y, image):
    if x < 0 or y < 0:
        return False
    if x > image.shape[1] - 1 or y > image.shape[0] - 1:
        return False
    if image[y, x] == 0:
        return False
    return True


@njit(cache=True)
def make_nodes_edges_index_map(image, do_diagonals=False):
    index_map = np.ones(image.shape, "int64") * -1

    yvals, xvals = np.where(image != 0)

    nodes = []
    i = 0
    for y, x in zip(yvals, xvals):
        if image[y, x] != 0:
            nodes.append((x, y))
            index_map[y, x] = i
            i += 1

    edges = []
    deltas = [(-1, 0), (0, -1), (1, 0), (0, 1)]
    if do_diagonals:
        deltas.extend([(1, 1), (-1, -1), (1, -1), (-1, 1)])
    for nx, ny in nodes:
        for dx, dy in deltas:
            x = nx + dx
            y = ny + dy
            if is_valid(x, y, image):
                edge = sorted([index_map[ny, nx], index_map[y, x]])
                edges.append(edge)

    return nodes, edges, index_map


def deduplicate_edges(edges):
    agg = {}
    for e in edges:
        agg[str(e)] = e
    return list(agg.values())


class AStarOnImage:
    def __init__(self, image, use_diagonals=False):
        self.nodes, self.edges, self.index_map = make_nodes_edges_index_map(
            image, use_diagonals
        )
        self.edges = deduplicate_edges(self.edges)
        self.astar = ac.AStar([ac.location(i[0], i[1]) for i in self.nodes], self.edges)
        self.image = image

    def plan_by_index(self, start: int, end: int):
        return [self.nodes[i] for i in self.astar.run(start, end)]

    def plan_by_coordinates(self, start: Tuple[int, int], end: Tuple[int, int]):
        # note: float values will be truncated
        p1, p2 = start, end
        p1 = [int(p1[0]), int(p1[1])]
        p2 = [int(p2[0]), int(p2[1])]
        assert is_valid(p1[0], p1[1], self.image), f"p1 {p1} is not a node in the graph"
        assert is_valid(p2[0], p2[1], self.image), f"p2 {p2} is not a node in the graph"
        return [
            self.nodes[i]
            for i in self.astar.run(
                self.index_map[p1[1], p1[0]], self.index_map[p2[1], p2[0]]
            )
        ]
