import math

import pytest

from office_delivery_robot.occupancy_grid import (
    L_FREE, L_HIT, L_MAX, L_MIN, OccupancyGrid, ray_cells, sensor_pose_from_body,
    sigmoid)


def make_grid():
    # 10 m x 10 m, 0.1 m cells, origin at (-5, -5)
    return OccupancyGrid(10.0, 10.0, resolution=0.1, origin_x=-5.0, origin_y=-5.0)


def test_world_to_cell_uses_floor():
    g = make_grid()
    assert g.world_to_cell(-5.0, -5.0) == (0, 0)
    assert g.world_to_cell(0.0, 0.0) == (50, 50)
    assert g.world_to_cell(0.099, 0.0) == (50, 50)
    assert g.world_to_cell(-0.01, 0.0) == (49, 50)   # floor, not truncation


def test_ray_is_horizontal():
    cols, rows = ray_cells(10, 20, 15, 20)
    assert list(cols) == [10, 11, 12, 13, 14, 15]
    assert set(rows) == {20}


def test_hit_and_free_increments():
    g = make_grid()
    ranges = [2.0]   # one beam straight ahead (+x)
    g.integrate_scan(0.0, 0.0, 0.0, ranges, 0.0, 0.01, 0.1, 10.0)
    c_end, r = g.world_to_cell(2.0, 0.0)
    c_mid, _ = g.world_to_cell(1.0, 0.0)
    assert g.log_odds[r, c_end] == pytest.approx(L_HIT)
    assert g.log_odds[r, c_mid] == pytest.approx(L_FREE)
    # one hit takes p from 0.5 to about 0.70
    assert sigmoid(L_HIT) == pytest.approx(0.70, abs=0.01)


def test_max_range_is_a_miss_not_a_hit():
    g = make_grid()
    g.integrate_scan(0.0, 0.0, 0.0, [10.0, math.inf], 0.0, 0.5, 0.1, 10.0)
    assert g.log_odds.max() <= 0.0          # no occupied cell anywhere
    assert g.log_odds.min() < 0.0           # but free space was recorded


def test_invalid_ranges_are_ignored():
    g = make_grid()
    stats = g.integrate_scan(0.0, 0.0, 0.0, [math.nan, -math.inf, 0.01],
                             0.0, 0.5, 0.12, 10.0)
    assert stats['ignored'] == 3
    assert not g.touched.any()


def test_clamp_limits():
    g = make_grid()
    for _ in range(50):
        g.integrate_scan(0.0, 0.0, 0.0, [2.0], 0.0, 0.01, 0.1, 10.0)
    c, r = g.world_to_cell(2.0, 0.0)
    assert g.log_odds[r, c] == pytest.approx(L_MAX)
    cm, rm = g.world_to_cell(1.0, 0.0)
    assert g.log_odds[rm, cm] == pytest.approx(L_MIN)


def test_sensor_offset_rotates_with_heading():
    xs, ys, _ = sensor_pose_from_body(1.0, 2.0, math.pi / 2, offset_x=0.10)
    assert xs == pytest.approx(1.0, abs=1e-9)   # NOT 1.10
    assert ys == pytest.approx(2.10)


def test_beam_angle_includes_robot_heading():
    g = make_grid()
    # robot faces +y, beam at angle 0 in sensor frame -> endpoint 2 m up
    g.integrate_scan(0.0, 0.0, math.pi / 2, [2.0], 0.0, 0.01, 0.1, 10.0)
    c, r = g.world_to_cell(0.0, 2.0)
    assert g.log_odds[r, c] == pytest.approx(L_HIT)


def test_image_flip_and_yaml(tmp_path):
    g = make_grid()
    g.integrate_scan(0.0, -4.0, 0.0, [2.0], 0.0, 0.01, 0.1, 10.0)
    img = g.to_image()
    c, r = g.world_to_cell(2.0, -4.0)
    # row 0 (lowest y) must be the LAST image row after flipping
    assert img[g.height - 1 - r, c] == 0
    g.save(str(tmp_path), 'm')
    text = (tmp_path / 'm.yaml').read_text()
    assert 'image: m.pgm' in text and 'resolution: 0.1' in text
    assert (tmp_path / 'm.pgm').exists()
