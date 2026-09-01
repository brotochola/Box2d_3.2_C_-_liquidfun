#!/usr/bin/env python3
"""Render liquidfun-c demo CSV frame dumps to PNG, for visual sanity
checking. Not part of the library - just a debug helper.

Usage:
  python render_frames.py [frames_dir] [out_dir]
Defaults: frames/ -> frames_png/
"""
import csv
import glob
import os
import sys

from PIL import Image, ImageDraw

WORLD_XMIN, WORLD_XMAX = -7.0, 7.0
WORLD_YMIN, WORLD_YMAX = -1.0, 6.0
IMG_W, IMG_H = 900, 550

BG = (18, 20, 28)
GROUND = (70, 70, 82)
WALL = (70, 70, 82)
RAMP = (90, 90, 105)
WATER = (70, 160, 235)
BOX = (235, 150, 60)


def world_to_px(x, y):
    u = (x - WORLD_XMIN) / (WORLD_XMAX - WORLD_XMIN)
    v = (y - WORLD_YMIN) / (WORLD_YMAX - WORLD_YMIN)
    return u * IMG_W, IMG_H - v * IMG_H


def draw_box(draw, cx, cy, hw, hh, angle, color):
    import math

    c, s = math.cos(angle), math.sin(angle)
    corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    pts = []
    for lx, ly in corners:
        wx = cx + lx * c - ly * s
        wy = cy + lx * s + ly * c
        pts.append(world_to_px(wx, wy))
    draw.polygon(pts, fill=color)


def render(csv_path, out_path, particle_radius=0.045):
    img = Image.new("RGB", (IMG_W, IMG_H), BG)
    draw = ImageDraw.Draw(img)

    draw_box(draw, 0.0, 0.0, 6.0, 0.5, 0.0, GROUND)
    draw_box(draw, -6.0, 5.0, 0.2, 5.0, 0.0, WALL)
    draw_box(draw, 6.0, 5.0, 0.2, 5.0, 0.0, WALL)
    draw_box(draw, -2.5, 1.0, 1.5, 0.15, 0.35, RAMP)

    box_pose = None
    particles = []
    with open(csv_path) as f:
        for row in csv.reader(f):
            if not row:
                continue
            if row[0] == "box":
                box_pose = (float(row[1]), float(row[2]), float(row[3]))
            elif row[0] == "p":
                particles.append((float(row[1]), float(row[2])))

    r_px = particle_radius / (WORLD_XMAX - WORLD_XMIN) * IMG_W
    for (x, y) in particles:
        px, py = world_to_px(x, y)
        draw.ellipse([px - r_px, py - r_px, px + r_px, py + r_px], fill=WATER)

    if box_pose:
        draw_box(draw, box_pose[0], box_pose[1], 0.3, 0.3, box_pose[2], BOX)

    label = os.path.basename(csv_path)
    draw.text((10, 10), f"{label}  particles={len(particles)}", fill=(230, 230, 230))

    img.save(out_path)


if __name__ == "__main__":
    frame_dir = sys.argv[1] if len(sys.argv) > 1 else "frames"
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "frames_png"
    os.makedirs(out_dir, exist_ok=True)
    for csv_path in sorted(glob.glob(os.path.join(frame_dir, "*.csv"))):
        out_path = os.path.join(out_dir, os.path.basename(csv_path).replace(".csv", ".png"))
        render(csv_path, out_path)
        print("wrote", out_path)
