"""Build functional map regions with seeded geodesic Voronoi partitioning.

This method partitions navigable space; it does not infer architectural rooms.
Unknown pixels, occupied pixels and map edges are treated as obstacles. Targets
are selected inside the largest connected component after clearance inflation.
"""

import argparse
from collections import deque
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import string

import cv2
import numpy as np
import yaml


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@dataclass
class OccupancyMap:
    """ROS trinary map plus conversions between image cells and map metres."""

    yaml_path: Path
    image_path: Path
    pixels: np.ndarray
    resolution: float
    origin: tuple
    free: np.ndarray
    occupied: np.ndarray

    @classmethod
    def from_yaml(cls, path):
        """Read map-server metadata; unsupported non-trinary modes are rejected."""
        path = Path(path).expanduser().resolve()
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8'))
            if not isinstance(data, dict):
                raise ValueError('Map YAML must be a mapping')
            if data.get('mode', 'trinary') != 'trinary':
                raise ValueError('Only ROS trinary maps are supported')
            image_path = Path(data['image']).expanduser()
            if not image_path.is_absolute():
                image_path = path.parent / image_path
            image_path = image_path.resolve()
            resolution = float(data['resolution'])
            origin = tuple(float(v) for v in data['origin'])
            free_thresh = float(data['free_thresh'])
            occupied_thresh = float(data['occupied_thresh'])
            negate = int(data['negate'])
            if (not math.isfinite(resolution) or resolution <= 0 or len(origin) != 3
                    or not all(math.isfinite(v) for v in origin)
                    or not 0 <= free_thresh < occupied_thresh <= 1
                    or negate not in (0, 1)):
                raise ValueError('Invalid resolution/origin/occupancy thresholds/negate')
            pixels = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
            if pixels is None or pixels.ndim != 2 or pixels.dtype != np.uint8:
                raise ValueError('Map image must be a readable 8-bit grayscale image')
            brightness = pixels.astype(np.float64) / 255.0
            probability = brightness if negate else 1.0 - brightness
            free = probability < free_thresh
            occupied = probability > occupied_thresh
            return cls(path, image_path, pixels, resolution, origin, free, occupied)
        except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
            raise ValueError(f'Cannot load map {path}: {exc}') from exc

    @property
    def height(self):
        return int(self.pixels.shape[0])

    @property
    def width(self):
        return int(self.pixels.shape[1])

    def pixel_to_world(self, col, row):
        """Return the centre of a top-left-indexed image cell in map coordinates."""
        if not 0 <= col < self.width or not 0 <= row < self.height:
            raise ValueError('Pixel lies outside the map')
        local_x = (float(col) + 0.5) * self.resolution
        local_y = (self.height - float(row) - 0.5) * self.resolution
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return ox + c * local_x - s * local_y, oy + s * local_x + c * local_y

    def world_to_pixel(self, x, y):
        """Return the containing image cell, rejecting points outside the map."""
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('Map coordinates must be finite')
        ox, oy, yaw = self.origin
        dx, dy = float(x) - ox, float(y) - oy
        c, s = math.cos(yaw), math.sin(yaw)
        local_x, local_y = c * dx + s * dy, -s * dx + c * dy
        col = math.floor(local_x / self.resolution)
        row = self.height - 1 - math.floor(local_y / self.resolution)
        if not 0 <= col < self.width or not 0 <= row < self.height:
            raise ValueError('Point lies outside the map')
        return int(col), int(row)

    def clearance(self):
        """Conservative Euclidean clearance to unknown/occupied cells and edges.

        Distance transform measures centre-to-centre distance. Subtract half a
        cell diagonal to bound distance to the full blocked cell, not its centre.
        A padded blocked border similarly keeps targets away from map edges.
        """
        padded = np.pad(self.free.astype(np.uint8), 1, constant_values=0)
        distances = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        return np.maximum(0.0, distances[1:-1, 1:-1] * self.resolution
                          - self.resolution * math.sqrt(2.0) / 2.0)

    def safe_mask(self, clearance_m=0.30):
        if not math.isfinite(clearance_m) or clearance_m <= 0:
            raise ValueError('clearance_m must be finite and positive')
        return self.free & (self.clearance() >= clearance_m)

    def fingerprint(self):
        return {'yaml_sha256': _sha256(self.yaml_path),
                'image_sha256': _sha256(self.image_path)}


def _largest_component(mask):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=4)
    if count <= 1:
        raise ValueError('No navigable cells remain after applying clearance')
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == largest


def _distances(mask, seed):
    """4-connected geodesic distance from one cell; no corner cutting."""
    h, w = mask.shape
    distances = np.full((h, w), -1, dtype=np.int32)
    col, row = seed
    distances[row, col] = 0
    queue = deque([(col, row)])
    while queue:
        col, row = queue.popleft()
        following = distances[row, col] + 1
        for nc, nr in ((col-1, row), (col+1, row), (col, row-1), (col, row+1)):
            if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and distances[nr, nc] < 0:
                distances[nr, nc] = following
                queue.append((nc, nr))
    return distances


def _choose_seeds(mask, clearance, count):
    """First seed has maximal obstacle clearance; later seeds are farthest apart."""
    if np.count_nonzero(mask) < count:
        raise ValueError('Fewer safe map cells than requested regions')
    row, col = np.unravel_index(np.argmax(np.where(mask, clearance, -1)), mask.shape)
    seeds = [(int(col), int(row))]
    nearest = _distances(mask, seeds[0])
    for _ in range(count-1):
        row, col = np.unravel_index(np.argmax(np.where(mask, nearest, -1)), mask.shape)
        if nearest[row, col] <= 0:
            raise ValueError('Cannot select distinct region seeds')
        seeds.append((int(col), int(row)))
        nearest = np.minimum(nearest, _distances(mask, seeds[-1]))
    return seeds


def _partition(mask, seeds):
    """Multi-source flood fill gives connected regions with deterministic ties."""
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    queue = deque()
    for label, (col, row) in enumerate(seeds, 1):
        labels[row, col] = label
        queue.append((col, row, label))
    while queue:
        col, row, label = queue.popleft()
        for nc, nr in ((col-1, row), (col+1, row), (col, row-1), (col, row+1)):
            if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and labels[nr, nc] == 0:
                labels[nr, nc] = label
                queue.append((nc, nr, label))
    return labels


def _draw_outputs(occupancy, labels, regions, output):
    """Write lossless numeric mask and a colour overlay with ASCII region IDs."""
    palette = [(80, 170, 245), (130, 210, 100), (230, 155, 125),
               (190, 120, 225), (90, 205, 225), (225, 205, 105)]
    base = cv2.cvtColor(occupancy.pixels, cv2.COLOR_GRAY2BGR)
    colour = np.zeros_like(base)
    overlay = base.copy()
    for region in regions:
        value = region['numeric_label']
        cells = labels == value
        rgb = palette[(value-1) % len(palette)]
        colour[cells] = rgb
        overlay[cells] = (base[cells] * 0.35 + np.array(rgb) * 0.65).astype(np.uint8)
        col, row = region['pixel']
        cv2.circle(overlay, (col, row), 5, (0, 0, 0), 2)
        cv2.putText(overlay, region['id'], (col+8, row+4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(colour, region['id'], (col+6, row+4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    if not cv2.imwrite(str(output / 'regions_labels.png'), colour):
        raise OSError('Cannot write region label image')
    if not cv2.imwrite(str(output / 'regions_overlay.png'), overlay):
        raise OSError('Cannot write region overlay image')
    np.savez_compressed(output / 'regions_mask.npz', labels=labels,
                        navigable=labels > 0)


def build_regions(map_yaml, output_dir, count=3, clearance_m=0.30):
    """Generate A/B/C functional regions, safe targets, masks and metadata."""
    if isinstance(count, bool) or not isinstance(count, int) or not 3 <= count <= 26:
        raise ValueError('Region count must be an integer from 3 to 26')
    occupancy = OccupancyMap.from_yaml(map_yaml)
    safe = _largest_component(occupancy.safe_mask(clearance_m))
    clearance = occupancy.clearance()
    seeds = _choose_seeds(safe, clearance, count)
    labels = _partition(safe, seeds)
    regions = []
    for index in range(1, count+1):
        cells = labels == index
        # Deepest interior of each safe region is a meaningful navigable target,
        # unlike an arithmetic centroid that might lie inside a shelf or wall.
        interior = cv2.distanceTransform(np.pad(cells.astype(np.uint8), 1),
                                         cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
        score = np.minimum(interior * occupancy.resolution, clearance)
        row, col = np.unravel_index(np.argmax(np.where(cells, score, -1)), cells.shape)
        x, y = occupancy.pixel_to_world(col, row)
        region_id = string.ascii_uppercase[index-1]
        aliases = [region_id, f'khu {region_id.lower()}', f'khu vực {region_id.lower()}',
                   f'khu {index}', f'khu vực {index}']
        if index <= 3:
            spoken_number = ('một', 'hai', 'ba')[index - 1]
            aliases.extend([f'khu {spoken_number}', f'khu vực {spoken_number}'])
        regions.append({'id': region_id, 'numeric_label': index,
                        'name': f'Khu vực {region_id}',
                        'aliases': aliases,
                        'goal': {'x': round(x, 6), 'y': round(y, 6), 'yaw': 0.0},
                        'pixel': [int(col), int(row)],
                        'area_m2': round(float(np.count_nonzero(cells))
                                          * occupancy.resolution**2, 4),
                        'clearance_m': round(float(clearance[row, col]), 4)})
    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    data = {'version': 1, 'frame_id': 'map', 'map_yaml': str(occupancy.yaml_path),
            'map_fingerprint': occupancy.fingerprint(), 'method': 'seeded_geodesic',
            'description': 'Functional regions of the largest clearance-safe component; '
                           'not automatic room classification.',
            'clearance_m': float(clearance_m), 'connectivity': 4,
            'mask_file': 'regions_mask.npz', 'regions': regions}
    _draw_outputs(occupancy, labels, regions, output)
    (output / 'regions.yaml').write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
    load_regions(output / 'regions.yaml')
    return data


def load_regions(path):
    """Load catalog; reject stale maps, duplicate IDs and invalid target cells."""
    path = Path(path).expanduser().resolve()
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
        if not isinstance(data, dict) or data.get('frame_id') != 'map':
            raise ValueError('Region catalog must use frame_id map')
        map_path = Path(data['map_yaml']).expanduser()
        if not map_path.is_absolute():
            map_path = path.parent / map_path
        occupancy = OccupancyMap.from_yaml(map_path)
        if data.get('map_fingerprint') != occupancy.fingerprint():
            raise ValueError('Map fingerprint mismatch: regenerate regions for this map')
        safe = _largest_component(occupancy.safe_mask(float(data['clearance_m'])))
        mask_path = Path(data['mask_file'])
        if not mask_path.is_absolute():
            mask_path = path.parent / mask_path
        with np.load(mask_path, allow_pickle=False) as stored:
            labels = stored['labels'].copy()
        if labels.shape != safe.shape or not np.issubdtype(labels.dtype, np.integer):
            raise ValueError('Invalid region mask shape/type')
        if np.any((labels > 0) & ~safe) or np.any(safe & (labels <= 0)):
            raise ValueError('Region mask must partition the largest clearance-safe component')
        regions = data['regions']
        if not isinstance(regions, list) or len(regions) < 3:
            raise ValueError('At least three regions are required')
        seen = set()
        expected_labels = set()
        for region in regions:
            region_id = region['id']
            numeric_label = region['numeric_label']
            if (not isinstance(region_id, str) or not region_id.strip() or region_id in seen
                    or not isinstance(numeric_label, int) or numeric_label <= 0
                    or numeric_label in expected_labels):
                raise ValueError('Region IDs and numeric labels must be unique')
            seen.add(region_id)
            expected_labels.add(numeric_label)
            if not isinstance(region['name'], str) or not region['name'].strip():
                raise ValueError('Region name must be non-empty')
            if not isinstance(region.get('aliases'), list) or not all(
                    isinstance(alias, str) and alias.strip() for alias in region['aliases']):
                raise ValueError('Region aliases must be non-empty strings')
            goal = region['goal']
            x, y, yaw = (float(goal[key]) for key in ('x', 'y', 'yaw'))
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError('Region goal must contain finite x/y/yaw')
            col, row = occupancy.world_to_pixel(x, y)
            if not safe[row, col] or labels[row, col] != numeric_label:
                raise ValueError(f'Region {region_id} target is unsafe or outside its region')
            if region.get('pixel') != [col, row]:
                raise ValueError(f'Region {region_id} pixel and world target disagree')
            components = cv2.connectedComponents(
                (labels == numeric_label).astype(np.uint8), connectivity=4)[0]
            if components != 2:
                raise ValueError(f'Region {region_id} must be one connected component')
        if set(np.unique(labels)) - {0} != expected_labels:
            raise ValueError('Region mask contains labels missing from the catalog')
        return data
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise ValueError(f'Cannot load region catalog {path}: {exc}') from exc


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, dest='map_yaml')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--regions', type=int, default=3)
    parser.add_argument('--clearance', type=float, default=0.30)
    options = parser.parse_args(args)
    try:
        data = build_regions(options.map_yaml, options.output_dir,
                             options.regions, options.clearance)
    except (ValueError, OSError) as exc:
        parser.exit(2, f'Cannot segment map: {exc}\n')
    for region in data['regions']:
        print(f"{region['id']}: {region['name']} -> {region['goal']}; "
              f"area={region['area_m2']:.2f} m2; clearance={region['clearance_m']:.2f} m")
    print(f"Saved region catalog and images to {Path(options.output_dir).resolve()}")


if __name__ == '__main__':
    main()
