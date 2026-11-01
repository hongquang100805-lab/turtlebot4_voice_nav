"""Regression tests for map transforms, safety margins and region catalogs."""

import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from turtlebot4_voice_nav.map_regions import OccupancyMap, build_regions, load_regions


def write_map(tmp_path, pixels=None, **updates):
    if pixels is None:
        pixels = np.full((80, 140), 254, dtype=np.uint8)
        pixels[:2] = pixels[-2:] = 0
        pixels[:, :2] = pixels[:, -2:] = 0
        pixels[20:60, 60:65] = 0
    image = tmp_path / 'map.pgm'
    cv2.imwrite(str(image), pixels)
    data = {'image': 'map.pgm', 'mode': 'trinary', 'resolution': 0.1,
            'origin': [-3.1, 2.7, 0.0], 'negate': 0,
            'occupied_thresh': 0.65, 'free_thresh': 0.196}
    data.update(updates)
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


def test_pixel_world_uses_centres_and_flips_image_rows(tmp_path):
    grid = OccupancyMap.from_yaml(write_map(tmp_path))
    assert grid.pixel_to_world(0, 0) == pytest.approx((-3.05, 10.65))
    assert grid.pixel_to_world(0, 79) == pytest.approx((-3.05, 2.75))
    for col, row in [(0, 0), (139, 79), (31, 42)]:
        assert grid.world_to_pixel(*grid.pixel_to_world(col, row)) == (col, row)


def test_rotated_origin_inverse_and_bounds(tmp_path):
    grid = OccupancyMap.from_yaml(write_map(tmp_path, origin=[8.0, -4.0, math.pi/2]))
    assert grid.pixel_to_world(0, 79) == pytest.approx((7.95, -3.95))
    for col, row in [(0, 0), (139, 79), (31, 42)]:
        assert grid.world_to_pixel(*grid.pixel_to_world(col, row)) == (col, row)
    with pytest.raises(ValueError):
        grid.world_to_pixel(1000, 1000)
    with pytest.raises(ValueError):
        grid.pixel_to_world(-1, 0)


def test_trinary_thresholds_and_negate(tmp_path):
    pixels = np.array([[0, 100, 205, 254, 255]], dtype=np.uint8)
    grid = OccupancyMap.from_yaml(write_map(tmp_path, pixels))
    assert grid.free.tolist() == [[False, False, False, True, True]]
    assert grid.occupied.tolist() == [[True, False, False, False, False]]
    other = OccupancyMap.from_yaml(write_map(tmp_path, pixels, negate=1))
    assert other.free.tolist() == [[True, False, False, False, False]]
    assert other.occupied.tolist() == [[False, False, True, True, True]]


def test_clearance_avoids_unknown_obstacles_and_edges(tmp_path):
    pixels = np.full((30, 30), 254, dtype=np.uint8)
    pixels[15, 15] = 205  # unknown must block just like occupied
    grid = OccupancyMap.from_yaml(write_map(tmp_path, pixels))
    safe = grid.safe_mask(0.30)
    assert not safe[0].any() and not safe[-1].any()
    assert not safe[:, 0].any() and not safe[:, -1].any()
    assert not safe[15, 15] and not safe[15, 17]
    assert safe[6, 6]
    assert np.all(grid.clearance()[safe] >= 0.30)
    with pytest.raises(ValueError):
        grid.safe_mask(float('nan'))


def test_generated_regions_are_connected_and_have_safe_goals(tmp_path):
    map_path = write_map(tmp_path)
    output = tmp_path / 'out'
    data = build_regions(map_path, output, 3, 0.30)
    assert [r['id'] for r in data['regions']] == ['A', 'B', 'C']
    assert data['method'] == 'seeded_geodesic'
    grid = OccupancyMap.from_yaml(map_path)
    labels = np.load(output / 'regions_mask.npz')['labels']
    for r in data['regions']:
        col, row = r['pixel']
        assert grid.world_to_pixel(r['goal']['x'], r['goal']['y']) == (col, row)
        assert grid.safe_mask(0.30)[row, col]
        assert r['clearance_m'] >= 0.30
        assert labels[row, col] == r['numeric_label']
        assert cv2.connectedComponents((labels == r['numeric_label']).astype(np.uint8), connectivity=4)[0] == 2
    assert load_regions(output / 'regions.yaml') == data
    assert (output / 'regions_overlay.png').is_file()
    assert (output / 'regions_labels.png').is_file()


def test_small_island_is_excluded_from_regions(tmp_path):
    pixels = np.zeros((80, 140), dtype=np.uint8)
    pixels[5:75, 5:100] = 254
    pixels[15:35, 115:135] = 254
    path = write_map(tmp_path, pixels)
    data = build_regions(path, tmp_path / 'out')
    labels = np.load(tmp_path / 'out/regions_mask.npz')['labels']
    assert np.all(labels[:, 110:] == 0)
    assert len(data['regions']) == 3


def test_stale_map_and_unsafe_goal_are_rejected(tmp_path):
    path = write_map(tmp_path)
    output = tmp_path / 'out'
    build_regions(path, output)
    catalog = output / 'regions.yaml'
    data = yaml.safe_load(catalog.read_text())
    data['regions'][0]['goal']['x'] = 1000
    catalog.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='outside'):
        load_regions(catalog)
    build_regions(path, output)
    image = cv2.imread(str(tmp_path / 'map.pgm'), cv2.IMREAD_GRAYSCALE)
    image[5, 5] = 0
    cv2.imwrite(str(tmp_path / 'map.pgm'), image)
    with pytest.raises(ValueError, match='fingerprint'):
        load_regions(catalog)


def test_blocked_map_and_invalid_region_count_fail(tmp_path):
    path = write_map(tmp_path, np.zeros((10, 10), dtype=np.uint8))
    with pytest.raises(ValueError, match='No navigable'):
        build_regions(path, tmp_path / 'out')
    with pytest.raises(ValueError):
        build_regions(path, tmp_path / 'out', count=2)


def test_regenerated_catalog_preserves_spoken_number_names(tmp_path):
    from turtlebot4_voice_nav.command_parser import parse_command
    data = build_regions(write_map(tmp_path), tmp_path / 'out')
    for number, word, identifier in [(1, 'một', 'A'), (2, 'hai', 'B'), (3, 'ba', 'C')]:
        assert parse_command(f'Đi đến khu vực {word}', data)['region_ids'] == [identifier]
        assert parse_command(f'Đi đến khu vực {number}', data)['region_ids'] == [identifier]
    with pytest.raises(ValueError):
        parse_command('Đi từ khu vực một đến khu vực chín', data)
