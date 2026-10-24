
#!/usr/bin/env python3

import osmium
import struct
import math
import sys
import os

PBF_FILE = "bulgaria-latest.osm.pbf"
OUT_FILE = "chernik.vec"

# Bbox: Dulovo, Silistra, Shumen, Varna
MIN_LAT = 43.10
MAX_LAT = 44.20
MIN_LON = 26.70
MAX_LON = 28.00

# ~5 m simplification tolerance
SIMPLIFY_TOL = 0.00005

TIER_LOCAL   = 0
TIER_MID     = 1
TIER_PRIMARY = 2
TIER_TRUNK   = 3
TIER_RAIL    = 4
TIER_WATER   = 5

KEEP_HIGHWAYS = {
    "motorway":       TIER_TRUNK,
    "motorway_link":  TIER_TRUNK,
    "trunk":          TIER_TRUNK,
    "trunk_link":     TIER_TRUNK,

    "primary":        TIER_PRIMARY,
    "primary_link":   TIER_PRIMARY,

    "secondary":      TIER_MID,
    "secondary_link": TIER_MID,
    "tertiary":       TIER_MID,
    "tertiary_link":  TIER_MID,

    "unclassified":   TIER_LOCAL,
    "residential":    TIER_LOCAL,
    "living_street":  TIER_LOCAL,
    "service":        TIER_LOCAL,
    "track":          TIER_LOCAL,
    "path":           TIER_LOCAL,
    "footway":        TIER_LOCAL,
}

KEEP_RAILWAYS = {
    "rail":         TIER_RAIL,
    "light_rail":   TIER_RAIL,
    "narrow_gauge": TIER_RAIL,
    "subway":       TIER_RAIL,
    "tram":         TIER_RAIL,
    "monorail":     TIER_RAIL,
}

# First waterway pass: linear water features only.
# Do not add riverbank here; it generally represents an area.
KEEP_WATERWAYS = {
    "river":  TIER_WATER,
    "stream": TIER_WATER,
    "canal":  TIER_WATER,
}


def in_bbox(lat, lon):
    return (
        MIN_LAT <= lat <= MAX_LAT
        and MIN_LON <= lon <= MAX_LON
    )


def perp_dist(p, a, b):
    """
    Perpendicular distance from point p to line segment a-b.
    Coordinates are latitude/longitude degrees.
    """
    x, y = p
    x1, y1 = a
    x2, y2 = b

    dx = x2 - x1
    dy = y2 - y1

    if dx == 0 and dy == 0:
        return math.hypot(x - x1, y - y1)

    t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))

    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return math.hypot(x - closest_x, y - closest_y)


def douglas_peucker(points, tol):
    """Simplify a line while preserving its endpoints."""
    if len(points) <= 2:
        return points

    first = points[0]
    last = points[-1]

    max_dist = 0.0
    max_index = 0

    for i in range(1, len(points) - 1):
        dist = perp_dist(points[i], first, last)
        if dist > max_dist:
            max_dist = dist
            max_index = i

    if max_dist > tol:
        left = douglas_peucker(points[:max_index + 1], tol)
        right = douglas_peucker(points[max_index:], tol)
        return left[:-1] + right

    return [first, last]


class RoadHandler(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.segments = []

    def way(self, w):
        tier = None

        # Existing road classification
        hw = w.tags.get("highway")
        if hw in KEEP_HIGHWAYS:
            tier = KEEP_HIGHWAYS[hw]

        # Existing railway classification
        if tier is None:
            rw = w.tags.get("railway")
            if rw in KEEP_RAILWAYS:
                tier = KEEP_RAILWAYS[rw]

        # New waterway classification
        if tier is None:
            ww = w.tags.get("waterway")
            if ww in KEEP_WATERWAYS:
                tier = KEEP_WATERWAYS[ww]

        if tier is None:
            return

        pts = []
        has_point_in_bbox = False

        for n in w.nodes:
            try:
                lat = n.location.lat
                lon = n.location.lon
            except osmium.InvalidLocationError:
                return

            if in_bbox(lat, lon):
                has_point_in_bbox = True

            pts.append((lat, lon))

        if not has_point_in_bbox:
            return

        if len(pts) < 2:
            return

        pts = douglas_peucker(pts, SIMPLIFY_TOL)

        if len(pts) >= 2:
            self.segments.append((tier, pts))


def main():
    if not os.path.isfile(PBF_FILE):
        print(f"ERROR: Input file not found: {PBF_FILE}")
        sys.exit(1)

    print(f"Reading {PBF_FILE} ...")

    handler = RoadHandler()
    handler.apply_file(PBF_FILE, locations=True)

    print(f"Found {len(handler.segments)} road, railway, and waterway segments")

    if not handler.segments:
        print("ERROR: No segments found.")
        sys.exit(1)

    lat_span = MAX_LAT - MIN_LAT
    lon_span = MAX_LON - MIN_LON

    lat_scale = max(1, int(30000 / lat_span))
    lon_scale = max(1, int(30000 / lon_span))

    print(f"Latitude scale:  {lat_scale}")
    print(f"Longitude scale: {lon_scale}")

    # Report segment counts by tier
    tier_counts = {}
    for tier, pts in handler.segments:
        tier_counts[tier] = tier_counts.get(tier, 0) + 1

    print("Segments by tier:")
    for tier in sorted(tier_counts):
        print(f"  Tier {tier}: {tier_counts[tier]}")

    with open(OUT_FILE, "wb") as f:
        # Header: VEC1 magic
        f.write(struct.pack("<I", 0x56454331))

        # Bounding box, integer coordinates × 10^7
        f.write(struct.pack("<i", int(MIN_LAT * 1e7)))
        f.write(struct.pack("<i", int(MIN_LON * 1e7)))
        f.write(struct.pack("<i", int(MAX_LAT * 1e7)))
        f.write(struct.pack("<i", int(MAX_LON * 1e7)))

        # Coordinate scales
        f.write(struct.pack("<I", lat_scale))
        f.write(struct.pack("<I", lon_scale))

        # Number of segments
        f.write(struct.pack("<I", len(handler.segments)))

        for tier, pts in handler.segments:
            # Each segment: uint8 tier, uint16 point count
            if len(pts) > 65535:
                print(f"WARNING: Skipping segment with {len(pts)} points")
                continue

            f.write(struct.pack("<B", tier))
            f.write(struct.pack("<H", len(pts)))

            for lat, lon in pts:
                qlat = int(round((lat - MIN_LAT) * lat_scale))
                qlon = int(round((lon - MIN_LON) * lon_scale))

                # Store as signed 16-bit values
                qlat = max(-32768, min(32767, qlat))
                qlon = max(-32768, min(32767, qlon))

                f.write(struct.pack("<h", qlat))
                f.write(struct.pack("<h", qlon))

    print(f"Wrote {OUT_FILE}: {os.path.getsize(OUT_FILE):,} bytes")


if __name__ == "__main__":
    main()
