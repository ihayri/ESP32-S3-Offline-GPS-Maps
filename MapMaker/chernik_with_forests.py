#!/usr/bin/env python3
"""Generate legacy chernik.vec plus a separate forests.vec polygon file.

Input: bulgaria-latest.osm.pbf
Dependency: pip install osmium

This first version extracts closed forest ways tagged landuse=forest or
natural=wood. It does not yet assemble multipolygon relations.
"""
import math
import os
import struct
import sys
import osmium

PBF_FILE = "bulgaria-latest.osm.pbf"
OUT_FILE = "chernik.vec"
FOREST_FILE = "forests.vec"

# Bbox: Dulovo, Silistra, Shumen, Varna
MIN_LAT, MAX_LAT = 43.10, 44.20
MIN_LON, MAX_LON = 26.70, 28.00
SIMPLIFY_TOL = 0.00005

TIER_LOCAL = 0
TIER_MID = 1
TIER_PRIMARY = 2
TIER_TRUNK = 3
TIER_RAIL = 4
TIER_WATER = 5

KEEP_HIGHWAYS = {
    "motorway": 3, "motorway_link": 3, "trunk": 3, "trunk_link": 3,
    "primary": 2, "primary_link": 2,
    "secondary": 1, "secondary_link": 1, "tertiary": 1, "tertiary_link": 1,
    "unclassified": 0, "residential": 0, "living_street": 0, "service": 0,
    "track": 0, "path": 0, "footway": 0,
}
KEEP_RAILWAYS = {
    "rail": 4, "light_rail": 4, "narrow_gauge": 4,
    "subway": 4, "tram": 4, "monorail": 4,
}
KEEP_WATERWAYS = {"river": 5, "stream": 5, "canal": 5}


def in_bbox(lat, lon):
    return MIN_LAT <= lat <= MAX_LAT and MIN_LON <= lon <= MAX_LON


def perp_dist(p, a, b):
    x, y = p
    x1, y1 = a
    x2, y2 = b
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(x - x1, y - y1)
    t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(x - (x1 + t * dx), y - (y1 + t * dy))


def douglas_peucker(points, tol):
    if len(points) <= 2:
        return points
    first, last = points[0], points[-1]
    max_dist, max_index = 0.0, 0
    for i in range(1, len(points) - 1):
        dist = perp_dist(points[i], first, last)
        if dist > max_dist:
            max_dist, max_index = dist, i
    if max_dist > tol:
        left = douglas_peucker(points[:max_index + 1], tol)
        right = douglas_peucker(points[max_index:], tol)
        return left[:-1] + right
    return [first, last]


def is_closed_forest_way(w):
    if len(w.nodes) < 4:
        return False
    if w.tags.get("landuse") != "forest" and w.tags.get("natural") != "wood":
        return False
    try:
        return w.nodes[0].ref == w.nodes[-1].ref
    except (IndexError, AttributeError):
        return False


class MapHandler(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.segments = []
        self.forests = []
        self.forest_skipped = 0

    def way(self, w):
        tier = None
        highway = w.tags.get("highway")
        if highway in KEEP_HIGHWAYS:
            tier = KEEP_HIGHWAYS[highway]
        if tier is None:
            railway = w.tags.get("railway")
            if railway in KEEP_RAILWAYS:
                tier = KEEP_RAILWAYS[railway]
        if tier is None:
            waterway = w.tags.get("waterway")
            if waterway in KEEP_WATERWAYS:
                tier = KEEP_WATERWAYS[waterway]

        if tier is not None:
            points, has_point_in_bbox = [], False
            for node in w.nodes:
                try:
                    lat, lon = node.location.lat, node.location.lon
                except osmium.InvalidLocationError:
                    return
                if in_bbox(lat, lon):
                    has_point_in_bbox = True
                points.append((lat, lon))
            if has_point_in_bbox and len(points) >= 2:
                points = douglas_peucker(points, SIMPLIFY_TOL)
                if len(points) >= 2:
                    self.segments.append((tier, points))

        if is_closed_forest_way(w):
            points, has_point_in_bbox = [], False
            for node in w.nodes:
                try:
                    lat, lon = node.location.lat, node.location.lon
                except osmium.InvalidLocationError:
                    self.forest_skipped += 1
                    return
                if in_bbox(lat, lon):
                    has_point_in_bbox = True
                points.append((lat, lon))
            if has_point_in_bbox and len(points) >= 4:
                points = douglas_peucker(points, SIMPLIFY_TOL)
                if len(points) >= 3 and points[0] != points[-1]:
                    points.append(points[0])
                if len(points) >= 4:
                    self.forests.append(points)


def quantize_point(lat, lon, lat_scale, lon_scale):
    qlat = int(round((lat - MIN_LAT) * lat_scale))
    qlon = int(round((lon - MIN_LON) * lon_scale))
    return max(-32768, min(32767, qlat)), max(-32768, min(32767, qlon))


def write_chernik_vec(path, segments):
    lat_scale = max(1, int(30000 / (MAX_LAT - MIN_LAT)))
    lon_scale = max(1, int(30000 / (MAX_LON - MIN_LON)))
    with open(path, "wb") as f:
        f.write(struct.pack("<I", 0x56454331))  # VEC1
        for value in (MIN_LAT, MIN_LON, MAX_LAT, MAX_LON):
            f.write(struct.pack("<i", int(value * 1e7)))
        f.write(struct.pack("<I", lat_scale))
        f.write(struct.pack("<I", lon_scale))
        f.write(struct.pack("<I", len(segments)))
        for tier, points in segments:
            if len(points) > 65535:
                print(f"WARNING: skipping line with {len(points)} points")
                continue
            f.write(struct.pack("<B", tier))
            f.write(struct.pack("<H", len(points)))
            for lat, lon in points:
                qlat, qlon = quantize_point(lat, lon, lat_scale, lon_scale)
                f.write(struct.pack("<hh", qlat, qlon))
    return lat_scale, lon_scale


def write_forests_vec(path, polygons, lat_scale, lon_scale):
    """FRST format: header like VEC1, then polygon_count, then uint16 n + qlat/qlon pairs."""
    with open(path, "wb") as f:
        f.write(struct.pack("<I", 0x54535246))  # bytes 'FRST'
        for value in (MIN_LAT, MIN_LON, MAX_LAT, MAX_LON):
            f.write(struct.pack("<i", int(value * 1e7)))
        f.write(struct.pack("<I", lat_scale))
        f.write(struct.pack("<I", lon_scale))
        f.write(struct.pack("<I", len(polygons)))
        for points in polygons:
            if len(points) > 65535:
                print(f"WARNING: skipping forest polygon with {len(points)} points")
                continue
            f.write(struct.pack("<H", len(points)))
            for lat, lon in points:
                qlat, qlon = quantize_point(lat, lon, lat_scale, lon_scale)
                f.write(struct.pack("<hh", qlat, qlon))


def main():
    if not os.path.isfile(PBF_FILE):
        print(f"ERROR: Input file not found: {PBF_FILE}")
        print("Put this script beside bulgaria-latest.osm.pbf.")
        sys.exit(1)

    print(f"Reading {PBF_FILE} ...")
    handler = MapHandler()
    handler.apply_file(PBF_FILE, locations=True)

    if not handler.segments:
        print("ERROR: No road, railway, or waterway segments found.")
        sys.exit(1)

    lat_scale, lon_scale = write_chernik_vec(OUT_FILE, handler.segments)
    write_forests_vec(FOREST_FILE, handler.forests, lat_scale, lon_scale)

    tier_counts = {}
    for tier, _ in handler.segments:
        tier_counts[tier] = tier_counts.get(tier, 0) + 1

    print(f"Wrote {OUT_FILE}: {os.path.getsize(OUT_FILE):,} bytes")
    print(f"Line segments: {len(handler.segments)}")
    print("Segments by tier:")
    for tier in sorted(tier_counts):
        print(f"  Tier {tier}: {tier_counts[tier]}")
    print(f"Wrote {FOREST_FILE}: {os.path.getsize(FOREST_FILE):,} bytes")
    print(f"Closed forest polygons: {len(handler.forests)}")
    if handler.forest_skipped:
        print(f"Forest ways skipped for missing coordinates: {handler.forest_skipped}")
    print("NOTE: forests.vec requires matching firmware code; current firmware does not read it.")
    print("NOTE: multipolygon forest relations are not yet assembled in this first version.")


if __name__ == "__main__":
    main()
