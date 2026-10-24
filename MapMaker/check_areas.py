import osmium

counts = {
    "areas": 0,
    "way_areas": 0,
    "relation_areas": 0,
    "forest_relation_areas": 0,
}

for obj in osmium.FileProcessor(
    "bulgaria-latest.osm.pbf"
).with_areas():

    if not obj.is_area():
        continue

    counts["areas"] += 1

    if obj.from_way():
        counts["way_areas"] += 1
    else:
        counts["relation_areas"] += 1

        if (
            obj.tags.get("landuse") == "forest"
            or obj.tags.get("natural") == "wood"
        ):
            counts["forest_relation_areas"] += 1

            if counts["forest_relation_areas"] <= 5:
                print(
                    "Forest relation area found:",
                    obj.orig_id(),
                    dict(obj.tags),
                    flush=True,
                )

print("RESULT:", counts, flush=True)