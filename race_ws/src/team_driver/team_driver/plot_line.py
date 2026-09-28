#!/usr/bin/env python3
"""Draw the map with the centreline, the v2.1 racing line and collision points.

Run inside the container, from /hackathon:
    python3 race_ws/src/team_driver/team_driver/plot_line.py --hits "5.54,1.52 1.72,14.35"

Writes results/line_plot.png (results/ is not committed, so this is safe).
Blue = centreline, red = racing line, grey = safe-corridor edges,
orange = racing-line points closer to a wall than the clearance, yellow X = hits.
"""
import argparse
import os
import sys
import types

import numpy as np
from PIL import Image, ImageDraw

# Import the path-planning functions without needing ROS.
for name in ['rclpy', 'rclpy.node', 'ackermann_msgs', 'ackermann_msgs.msg', 'geometry_msgs',
             'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg',
             'visualization_msgs', 'visualization_msgs.msg']:
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules['rclpy.node'].Node = object
for mod, attrs in {'ackermann_msgs.msg': ['AckermannDriveStamped'], 'geometry_msgs.msg': ['Point'],
                   'nav_msgs.msg': ['Odometry'], 'sensor_msgs.msg': ['LaserScan'],
                   'visualization_msgs.msg': ['Marker', 'MarkerArray']}.items():
    for a in attrs:
        setattr(sys.modules[mod], a, object)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import racing_driver as rd  # noqa: E402
from scipy.ndimage import distance_transform_edt  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--margin', type=float, default=rd.V21_SETTINGS['line_margin'])
ap.add_argument('--length-weight', type=float, default=rd.V21_SETTINGS['length_weight'])
ap.add_argument('--hits', default='', help='collision points "x,y x,y ..."')
ap.add_argument('--scale', type=int, default=3)
ap.add_argument('--out', default='results/line_plot.png')
args = ap.parse_args()

center, _ = rd.load_path('')
img, res, origin, _ = rd.load_map('')
clearance = 0.35 + args.margin
line, info = rd.compute_racing_line(center, '', clearance, 0.2, args.length_weight)
dist = distance_transform_edt(img >= 128) * res

h, w = img.shape
s = args.scale
canvas = Image.fromarray(img).convert('RGB').resize((w * s, h * s), Image.NEAREST)
draw = ImageDraw.Draw(canvas)


def px(xy):
    return [((x - origin[0]) / res * s, (h - (y - origin[1]) / res) * s) for x, y in xy]


def wall_distance(xy):
    col = np.clip(((xy[:, 0] - origin[0]) / res).astype(int), 0, w - 1)
    row = np.clip(h - 1 - ((xy[:, 1] - origin[1]) / res).astype(int), 0, h - 1)
    return dist[row, col]


for pts, colour, width in ((info['left'], (150, 150, 150), 1), (info['right'], (150, 150, 150), 1),
                           (center, (40, 90, 255), 2), (line, (230, 30, 30), 2)):
    p = px(np.vstack([pts, pts[:1]]))
    draw.line(p, fill=colour, width=width)

close = line[wall_distance(line) < clearance]
for x, y in px(close):
    draw.ellipse([x - 4, y - 4, x + 4, y + 4], outline=(255, 140, 0), width=2)

if args.hits:
    for token in args.hits.split():
        hx, hy = (float(v) for v in token.split(','))
        (x, y), = px([(hx, hy)])
        draw.line([x - 8, y - 8, x + 8, y + 8], fill=(255, 220, 0), width=3)
        draw.line([x - 8, y + 8, x + 8, y - 8], fill=(255, 220, 0), width=3)

# Axis ticks every 2 m, so positions in the log can be found on the picture
for gx in range(int(np.ceil(origin[0])), int(origin[0] + w * res) + 1, 2):
    (x, _), = px([(gx, origin[1])])
    draw.text((x + 2, 2), f'x={gx}', fill=(0, 150, 0))
    draw.line([x, 0, x, 10], fill=(0, 150, 0))
for gy in range(int(np.ceil(origin[1])), int(origin[1] + h * res) + 1, 2):
    (_, y), = px([(origin[0], gy)])
    draw.text((2, y - 10), f'y={gy}', fill=(0, 150, 0))
    draw.line([0, y, 10, y], fill=(0, 150, 0))

os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
canvas.save(args.out)
print(f'margin {args.margin}, length_weight {args.length_weight}: line {info["max_offset"]:.2f} m max offset, '
      f'pinned {info["pinned"]}, unsafe {info["unsafe"]}, '
      f'{len(close)} line points closer than {clearance:.2f} m to a wall.')
print(f'Minimum wall distance along the centreline: {wall_distance(center).min():.2f} m, '
      f'along the racing line: {wall_distance(line).min():.2f} m.')
print('Saved', args.out)
