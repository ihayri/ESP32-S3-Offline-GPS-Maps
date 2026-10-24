#!/usr/bin/env python3
"""Generate chernik.vec and forests.vec from Bulgaria's OSM PBF.

Road/rail/water output keeps the existing VEC1 format unchanged.
Forest output keeps the existing FRST format unchanged and adds outer rings
from forest multipolygon relations, in addition to closed forest ways.

Requirements:
    pip install osmium

Put this script beside bulgaria-latest.osm.pbf and run it with Python.
NOTE: FRST stores one ring per polygon and cannot represent holes. Inner rings
of multipolygon relations are intentionally not written, so lakes/clearings
inside a forest may be filled by the current firmware.
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


def forest_tagged(tags):
    return tags.get("landuse") == "forest" or tags.get("natural") == "wood"


def clean_forest_ring(points):
    """Clip by bbox intersection, simplify, and return a closed ring or None."""
    if len(points) < 4 or not any(in_bbox(lat, lon) for lat, lon in points):
        return None
    points = douglas_peucker(points, SIMPLIFY_TOL)
    if len(points) >= 3 and points[0] != points[-1]:
        points.append(points[0])
    if len(points) < 4:
        return None
    return points


class MapHandler(osmium.SimpleHandler):
    """First pass: unchanged road/rail/water extraction plus tagged closed ways."""
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
            points = []
            try:
                for node in w.nodes:
                    points.append((node.location.lat, node.location.lon))
            except osmium.InvalidLocationError:
                self.forest_skipped += 1
                return
            ring = clean_forest_ring(points)
            if ring:
                self.forests.append(ring)


class ForestRelationCounter:
    """Collect forest relation outer rings using libosmium's area assembler."""
    def __init__(self):
        self.polygons = []
        self.relations_seen = 0
        self.rings_skipped = 0

    def collect(self):
        # FileProcessor assembles multipolygon relations into Area objects.
        # Location indexing is needed to obtain coordinates for ring nodes.
        processor = osmium.FileProcessor(PBF_FILE).with_locations().with_areas()
        for area in processor:
            # FileProcessor also yields ordinary nodes and ways. Use the
            # documented Area API to identify actual assembled area objects.
            if not area.is_area():
                continue
            try:
                # Closed ways are already collected in MapHandler; here we
                # only want multipolygon relation areas to avoid duplication.
                if area.from_way():
                    continue
                if not forest_tagged(area.tags):
                    continue
                self.relations_seen += 1
                for ring in area.outer_rings():
                    points = []
                    try:
                        for node in ring:
                            points.append((node.lat, node.lon))
                    except (osmium.InvalidLocationError, AttributeError):
                        self.rings_skipped += 1
                        continue
                    cleaned = clean_forest_ring(points)
                    if cleaned:
                        self.polygons.append(cleaned)
                    else:
                        self.rings_skipped += 1
            except (RuntimeError, ValueError, AttributeError):
                self.rings_skipped += 1
        return self


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
        written = 0
        for tier, points in segments:
            if len(points) > 65535:
                print(f"WARNING: skipping line with {len(points)} points")
                continue
            f.write(struct.pack("<B", tier))
            f.write(struct.pack("<H", len(points)))
            for lat, lon in points:
                qlat, qlon = quantize_point(lat, lon, lat_scale, lon_scale)
                f.write(struct.pack("<hh", qlat, qlon))
            written += 1
    return lat_scale, lon_scale, written


def write_forests_vec(path, polygons, lat_scale, lon_scale):
    """Write the existing FRST format expected by the current firmware."""
    with open(path, "wb") as f:
        f.write(struct.pack("<I", 0x54535246))  # FRST
        for value in (MIN_LAT, MIN_LON, MAX_LAT, MAX_LON):
            f.write(struct.pack("<i", int(value * 1e7)))
        f.write(struct.pack("<I", lat_scale))
        f.write(struct.pack("<I", lon_scale))
        f.write(struct.pack("<I", len(polygons)))
        written = 0
        for points in polygons:
            if len(points) > 65535:
                print(f"WARNING: skipping forest polygon with {len(points)} points")
                continue
            f.write(struct.pack("<H", len(points)))
            for lat, lon in points:
                qlat, qlon = quantize_point(lat, lon, lat_scale, lon_scale)
                f.write(struct.pack("<hh", qlat, qlon))
            written += 1
    return written


def main():
    if not os.path.isfile(PBF_FILE):
        print(f"ERROR: Input file not found: {PBF_FILE}")
        print("Put this script beside bulgaria-latest.osm.pbf.")
        sys.exit(1)

    print(f"Reading roads and closed forest ways from {PBF_FILE} ...")
    handler = MapHandler()
    handler.apply_file(PBF_FILE, locations=True)

    if not handler.segments:
        print("ERROR: No road, railway, or waterway segments found.")
        sys.exit(1)

    print("Assembling forest multipolygon relations (second pass)...")
    try:
        relation_collector = ForestRelationCounter().collect()
    except Exception as exc:
        print("ERROR: multipolygon area assembly failed.")
        print(f"Details: {exc}")
        print("No output files have been written. Check your pyosmium version.")
        sys.exit(1)

    way_forests = handler.forests
    relation_forests = relation_collector.polygons
    all_forests = way_forests + relation_forests

    lat_scale, lon_scale, written_segments = write_chernik_vec(OUT_FILE, handler.segments)
    written_forests = write_forests_vec(FOREST_FILE, all_forests, lat_scale, lon_scale)

    tier_counts = {}
    for tier, _ in handler.segments:
        tier_counts[tier] = tier_counts.get(tier, 0) + 1

    print(f"\nWrote {OUT_FILE}: {os.path.getsize(OUT_FILE):,} bytes")
    print(f"Road/rail/water segments collected: {len(handler.segments):,}")
    print(f"Road/rail/water segments written:   {written_segments:,}")
    print("Segments by tier:")
    for tier in sorted(tier_counts):
        print(f"  Tier {tier}: {tier_counts[tier]:,}")

    print(f"\nWrote {FOREST_FILE}: {os.path.getsize(FOREST_FILE):,} bytes")
    print(f"Closed forest-way polygons:         {len(way_forests):,}")
    print(f"Forest multipolygon relations seen: {relation_collector.relations_seen:,}")
    print(f"Outer rings from relations:         {len(relation_forests):,}")
    print(f"Forest rings skipped:               {handler.forest_skipped + relation_collector.rings_skipped:,}")
    print(f"Total forest polygons collected:    {len(all_forests):,}")
    print(f"Forest polygons written:            {written_forests:,}")
    print("\nNOTE: Inner rings/holes are not represented by the current FRST format.")
    print("NOTE: Relation outer rings are written as separate filled polygons.")


if __name__ == "__main__":
    main()
