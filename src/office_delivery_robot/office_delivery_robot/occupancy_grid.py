"""Log-odds occupancy grid (Lab 3 mapper, adapted for office_delivery_robot).

Pure Python + NumPy: no ROS imports, so it can be unit-tested anywhere.

Conventions (same as Lab 3):
  * cell column = floor((x - origin_x) / resolution), row likewise with y.
  * Rows increase upward in world y. Image files store the top row first,
    so ``to_image`` flips the rows before export.
  * Log odds: l = log(p / (1 - p)); p = 0 .5 -> l = 0 (unknown / no evidence).
  * Hit adds L_HIT, free-space observation adds L_FREE, result clamped.
  * A range at the sensor maximum (or +inf) is a MISS: free space is
    recorded along the ray but no occupied endpoint is created.
  * Invalid ranges (NaN, -inf, below range_min) are ignored.
"""

import json
import math
import os

import numpy as np

L_HIT = 0.85      # evidence added to the endpoint cell of a valid return
L_FREE = -0.40    # evidence added to cells the beam passed through
L_MIN = -5.0      # clamp so old evidence can always be overturned
L_MAX = 5.0

OCCUPIED_THRESH = 0.65   # same names/values as nav2 map_server YAML
FREE_THRESH = 0.25

# Pixel values used by nav2_map_server's map_saver (trinary mode).
PIXEL_FREE = 254
PIXEL_UNKNOWN = 205
PIXEL_OCCUPIED = 0


def logit(p):
    """Probability -> log odds."""
    return math.log(p / (1.0 - p))


def sigmoid(l_value):
    """Log odds -> probability (works on floats and NumPy arrays)."""
    return 1.0 / (1.0 + np.exp(-l_value))


def sensor_pose_from_body(xb, yb, yaw_b, offset_x=0.10, offset_y=0.0):
    """World pose of the laser given the body pose and the mount offset.

    The offset is rotated by the body heading. Adding 0.10 to world x
    regardless of heading would be wrong as soon as the robot turns.
    """
    c, s = math.cos(yaw_b), math.sin(yaw_b)
    xs = xb + offset_x * c - offset_y * s
    ys = yb + offset_x * s + offset_y * c
    return xs, ys, yaw_b


def ray_cells(c0, r0, c1, r1):
    """Cells on the line (c0, r0) -> (c1, r1), both ends included.

    Vectorised equivalent of Bresenham: one cell per step along the major
    axis. Returns two integer arrays (cols, rows).
    """
    dc, dr = c1 - c0, r1 - r0
    steps = max(abs(dc), abs(dr))
    if steps == 0:
        return np.array([c0]), np.array([r0])
    t = np.arange(steps + 1) / steps
    cols = np.floor(c0 + dc * t + 0.5).astype(int)
    rows = np.floor(r0 + dr * t + 0.5).astype(int)
    return cols, rows


class OccupancyGrid:
    """Occupancy grid stored as a float array of log odds."""

    def __init__(self, width_m, height_m, resolution=0.05,
                 origin_x=0.0, origin_y=0.0):
        if resolution <= 0.0:
            raise ValueError('resolution must be positive')
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.width = int(math.ceil(width_m / resolution))    # columns (x)
        self.height = int(math.ceil(height_m / resolution))  # rows (y)
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float64)
        self.touched = np.zeros((self.height, self.width), dtype=bool)

    # ---- coordinates -------------------------------------------------
    def world_to_cell(self, x, y):
        """Return (col, row) for a world point; no bounds check."""
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return col, row

    def cell_to_world(self, col, row):
        """World coordinates of a cell centre."""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, col, row):
        """True when the cell lies inside the grid."""
        return 0 <= col < self.width and 0 <= row < self.height

    # ---- evidence updates --------------------------------------------
    def _update_cell(self, col, row, delta):
        """Add evidence to one cell and clamp (ignored if out of bounds)."""
        if not self.in_bounds(col, row):
            return
        value = self.log_odds[row, col] + delta
        self.log_odds[row, col] = min(L_MAX, max(L_MIN, value))
        self.touched[row, col] = True

    def _update_cells(self, cols, rows, delta):
        """Vectorised evidence update for many cells (duplicates add up)."""
        ok = ((cols >= 0) & (cols < self.width)
              & (rows >= 0) & (rows < self.height))
        cols, rows = cols[ok], rows[ok]
        if cols.size == 0:
            return
        np.add.at(self.log_odds, (rows, cols), delta)
        self.touched[rows, cols] = True

    def integrate_scan(self, sensor_x, sensor_y, sensor_yaw, ranges,
                       angle_min, angle_increment, range_min, range_max):
        """Integrate one LaserScan taken from a known sensor pose.

        Returns a dict with counts of hits / misses / ignored beams.
        """
        stats = {'hits': 0, 'misses': 0, 'ignored': 0}
        c0, r0 = self.world_to_cell(sensor_x, sensor_y)
        if not self.in_bounds(c0, r0):
            stats['ignored'] = len(ranges)
            return stats

        for i, d in enumerate(ranges):
            if d != d or d == -math.inf or d < range_min:
                stats['ignored'] += 1          # NaN, -inf, too close
                continue
            is_miss = d == math.inf or d >= range_max
            dist = range_max if is_miss else d
            if not is_miss and not math.isfinite(d):
                stats['ignored'] += 1
                continue

            angle = sensor_yaw + angle_min + i * angle_increment
            xe = sensor_x + dist * math.cos(angle)
            ye = sensor_y + dist * math.sin(angle)
            c1, r1 = self.world_to_cell(xe, ye)
            cols, rows = ray_cells(c0, r0, c1, r1)

            if is_miss:
                self._update_cells(cols, rows, L_FREE)
                stats['misses'] += 1
            else:
                self._update_cells(cols[:-1], rows[:-1], L_FREE)
                self._update_cells(cols[-1:], rows[-1:], L_HIT)
                stats['hits'] += 1

        np.clip(self.log_odds, L_MIN, L_MAX, out=self.log_odds)
        return stats

    # ---- export --------------------------------------------------------
    def probability(self):
        """Occupancy probability per cell (0.5 where nothing was seen)."""
        return sigmoid(self.log_odds)

    def to_image(self):
        """uint8 image in nav2 trinary convention, top row first."""
        img = np.full(self.log_odds.shape, PIXEL_UNKNOWN, dtype=np.uint8)
        l_occ = logit(OCCUPIED_THRESH)
        l_free = logit(FREE_THRESH)
        img[self.touched & (self.log_odds >= l_occ)] = PIXEL_OCCUPIED
        img[self.touched & (self.log_odds <= l_free)] = PIXEL_FREE
        return np.flipud(img)    # rows increase upward, images start at top

    def save(self, out_dir, stem='map'):
        """Write <stem>.pgm, <stem>.yaml, <stem>.png and <stem>_logodds.npy.

        The YAML/PGM pair is what nav2 map_server loads in later labs.
        """
        os.makedirs(out_dir, exist_ok=True)
        img = self.to_image()
        pgm_path = os.path.join(out_dir, stem + '.pgm')
        yaml_path = os.path.join(out_dir, stem + '.yaml')

        with open(pgm_path, 'wb') as f:
            f.write(b'P5\n# office_delivery_robot occupancy grid\n')
            f.write(f'{img.shape[1]} {img.shape[0]}\n255\n'.encode())
            f.write(img.tobytes())

        with open(yaml_path, 'w') as f:
            f.write(f'image: {stem}.pgm\n')
            f.write('mode: trinary\n')
            f.write(f'resolution: {self.resolution}\n')
            f.write(f'origin: [{self.origin_x}, {self.origin_y}, 0.0]\n')
            f.write('negate: 0\n')
            f.write(f'occupied_thresh: {OCCUPIED_THRESH}\n')
            f.write(f'free_thresh: {FREE_THRESH}\n')

        np.save(os.path.join(out_dir, stem + '_logodds.npy'), self.log_odds)

        try:    # PNG is only a convenience for quick viewing
            from PIL import Image
            Image.fromarray(img).save(os.path.join(out_dir, stem + '.png'))
        except ImportError:
            pass
        return pgm_path, yaml_path

    def summary(self):
        """Small dict for report.json."""
        l_occ = logit(OCCUPIED_THRESH)
        l_free = logit(FREE_THRESH)
        return {
            'width_cells': self.width,
            'height_cells': self.height,
            'resolution_m': self.resolution,
            'origin': [self.origin_x, self.origin_y],
            'occupied_cells': int(np.sum(self.touched & (self.log_odds >= l_occ))),
            'free_cells': int(np.sum(self.touched & (self.log_odds <= l_free))),
        }


def write_report(path, data):
    """Dump a dict as pretty JSON."""
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
