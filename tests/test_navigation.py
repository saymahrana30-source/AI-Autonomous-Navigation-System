import dataclasses
import numpy as np
from src.config import Config
from src.perception import OccupancyMap
from src.sensors import Lidar
from src.simulation import Simulation
from src.world import build_scenario


def test_lidar_measures_distance_to_wall():
    cfg = Config(range_noise=0.0)
    occ = np.zeros((20, 20), dtype=bool); occ[:, 11] = True
    scan = Lidar(cfg).scan((5.5, 10.5, 0.0), occ, np.random.default_rng(0))
    front = scan.ranges[np.argmin(np.abs(scan.rel_angles))]
    assert abs(front - 5.5) < 0.2        # wall starts 5.5 cells ahead


def test_mapping_adds_hidden_obstacle_and_removes_it_when_it_leaves():
    cfg = Config(range_noise=0.0)
    prior = np.zeros((20, 20), dtype=bool)
    truth = prior.copy(); truth[10, 12] = True
    m, lidar, rng = OccupancyMap(prior, cfg), Lidar(cfg), np.random.default_rng(0)
    pose = (6.5, 10.5, 0.0)
    for _ in range(3):
        m.update(pose, lidar.scan(pose, truth, rng))
    assert m.occupied()[10, 12]
    truth[10, 12] = False
    for _ in range(4):
        m.update(pose, lidar.scan(pose, truth, rng))
    assert not m.occupied()[10, 12]


def test_warehouse_reaches_goal_without_collision():
    m = Simulation(build_scenario("warehouse")).run()
    assert m["success"] and m["collisions"] == 0


def test_city_reaches_goal_without_collision():
    m = Simulation(build_scenario("city")).run()
    assert m["success"] and m["collisions"] == 0


def test_works_without_prior_map():
    m = Simulation(build_scenario("warehouse"), use_prior=False).run()
    assert m["success"] and m["collisions"] == 0


def test_random_maps_are_navigable():
    for seed in range(5):
        m = Simulation(build_scenario("random", seed)).run()
        assert m["success"] and m["collisions"] == 0, f"seed {seed}: {m}"


def test_safety_layer_stops_robot_before_obstacle():
    sim = Simulation(build_scenario("warehouse"))
    a = sim.agent
    sim.step()                                   # plan once
    # put a wall 0.6 m in front of the robot and bypass the planner
    occ = np.zeros_like(sim.world.occ)
    x, y, th = a.pose
    occ[int(y), int(x + 0.6)] = True
    scan = a.lidar.scan(a.pose, occ, np.random.default_rng(0))
    a.perceive(scan)
    v, w = a.act(1)
    assert v == 0.0 and a.state.value == "AVOIDING"
