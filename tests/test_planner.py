import numpy as np
from src.planner import plan, smooth_path, line_of_sight
from src.utils import inflate


def _grid():
    g = np.zeros((20, 20), dtype=bool)
    g[5:15, 10] = True          # a wall with gaps at the top and bottom
    return g


def test_astar_finds_path_that_avoids_obstacles():
    g = _grid()
    res = plan(g, (10, 2), (10, 18))
    assert res.success and res.path[0] == (10, 2) and res.path[-1] == (10, 18)
    assert not any(g[r, c] for r, c in res.path)


def test_astar_matches_dijkstra_cost_but_expands_fewer_nodes():
    g = _grid()
    a, d = plan(g, (10, 2), (10, 18), "astar"), plan(g, (10, 2), (10, 18), "dijkstra")
    assert abs(a.cost - d.cost) < 1e-9
    assert a.expanded < d.expanded


def test_no_path_when_goal_enclosed():
    g = np.zeros((10, 10), dtype=bool)
    g[3:6, 3:6] = True
    g[4, 4] = False             # goal sealed inside a box
    assert not plan(g, (0, 0), (4, 4)).success


def test_goal_inside_obstacle_fails():
    g = np.zeros((5, 5), dtype=bool); g[2, 2] = True
    assert not plan(g, (0, 0), (2, 2)).success


def test_no_diagonal_corner_cutting():
    g = np.zeros((3, 3), dtype=bool)
    g[0, 1] = g[1, 0] = True    # diagonal squeeze between two blocks
    assert not plan(g, (0, 0), (1, 1)).success


def test_smoothing_keeps_path_collision_free_and_shorter():
    g = _grid()
    res = plan(g, (10, 2), (2, 18))
    sm = smooth_path(g, res.path)
    assert len(sm) <= len(res.path)
    for a, b in zip(sm[:-1], sm[1:]):
        assert line_of_sight(g, a, b)


def test_inflation_grows_obstacles():
    g = np.zeros((7, 7), dtype=bool); g[3, 3] = True
    assert inflate(g, 1).sum() == 9
