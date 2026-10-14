#!/usr/bin/env python3
"""
Convert Moving AI (MAPF benchmark) .map / .scen files into a custom format,
with one subfolder per map.

Usage:
    python convert_mapf.py MAPS_DIR SCEN_DIR OUTPUT_DIR

Output layout:
    OUTPUT_DIR/
        Berlin_1_256/
            map_Berlin_1_256.txt
            scen_Berlin_1_256-random-1.txt
            scen_Berlin_1_256-random-2.txt
            scen_Berlin_1_256-even-1.txt
            ...
        Paris_1_256/
            map_Paris_1_256.txt
            scen_Paris_1_256-random-1.txt
            ...

Each scenario is placed in the folder of the map it refers to (taken from the
map column inside the .scen file; if that map isn't found, the scenario's
filename is matched against the map names instead).

Map format written:
    # width height depth
    W H 1
    # obstacles (O x y z)
    # pads (P x y z)
    O x y 0

Scenario format written:
    # start_x start_y start_z goal_x goal_y goal_z delay start_time
    sx sy 0 gx gy 0 0 0
"""

import argparse
import os
import sys

# Terrain characters treated as obstacles. Everything else (., G, S, ...) is free.
#   @ / O : out of bounds
#   T     : trees
#   W     : water (not passable from regular terrain, so treated as blocked).
#           Remove "W" from this set if you want water to count as free.
OBSTACLE_CHARS = {"@", "O", "T", "W"}


def find_files(root, extension):
    """Recursively find files with a given extension under root."""
    found = []
    for dirpath, _, filenames in os.walk(root):
        for name in sorted(filenames):
            if name.lower().endswith(extension):
                found.append(os.path.join(dirpath, name))
    return sorted(found)


def parse_map(path):
    """Return (width, height, grid) where grid is a list of row strings."""
    width = height = None
    grid_start = None

    with open(path, "r") as f:
        lines = f.read().splitlines()

    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("height"):
            height = int(stripped.split()[1])
        elif stripped.startswith("width"):
            width = int(stripped.split()[1])
        elif stripped == "map":
            grid_start = i + 1
            break

    if width is None or height is None or grid_start is None:
        raise ValueError("missing width/height/map header")

    grid = [line.rstrip("\r\n") for line in lines[grid_start:grid_start + height]]

    if len(grid) != height:
        print(f"  Warning: {path} declares height {height} but has {len(grid)} rows")
    for y, row in enumerate(grid):
        if len(row) != width:
            print(f"  Warning: {path} row {y} has length {len(row)}, expected {width}")
            break

    return width, height, grid


def convert_map(in_path, out_path):
    """Convert one map. Returns (width, height, set of blocked (x, y))."""
    width, height, grid = parse_map(in_path)
    blocked = set()

    with open(out_path, "w") as f:
        f.write("# width height depth\n")
        f.write(f"{width} {height} 1\n")
        f.write("# obstacles (O x y z)\n")
        f.write("# pads (P x y z)\n")
        for y, row in enumerate(grid):
            for x, ch in enumerate(row):
                if ch in OBSTACLE_CHARS:
                    blocked.add((x, y))
                    f.write(f"O {x} {y} 0\n")

    return width, height, blocked


def parse_scen(path):
    """Return (map_stem, agents) where agents is a list of (sx, sy, gx, gy)."""
    agents = []
    map_stem = None

    with open(path, "r") as f:
        for line in f:
            parts = line.split()
            if not parts or parts[0].lower().startswith("version"):
                continue
            if len(parts) < 9:
                print(f"  Warning: skipping malformed line in {path}: {line.strip()!r}")
                continue

            # Columns: bucket, map, map_w, map_h, sx, sy, gx, gy, optimal_length
            if map_stem is None:
                map_stem = os.path.splitext(os.path.basename(parts[1]))[0]
            agents.append((int(parts[4]), int(parts[5]), int(parts[6]), int(parts[7])))

    return map_stem, agents


def match_map_by_filename(scen_stem, map_names):
    """Fallback: pick the longest map name that prefixes the scenario filename."""
    best = None
    for m in map_names:
        if scen_stem == m or scen_stem.startswith(m + "-"):
            if best is None or len(m) > len(best):
                best = m
    return best


def check_agents(scen_name, agents, map_data):
    """Print warnings for out-of-bounds, blocked or duplicate positions."""
    width, height, blocked = map_data
    for i, (sx, sy, gx, gy) in enumerate(agents):
        for label, (x, y) in (("start", (sx, sy)), ("goal", (gx, gy))):
            if not (0 <= x < width and 0 <= y < height):
                print(f"  Warning: {scen_name} agent {i} {label} ({x},{y}) is out of bounds")
            elif (x, y) in blocked:
                print(f"  Warning: {scen_name} agent {i} {label} ({x},{y}) is on an obstacle")
    if len({(a[0], a[1]) for a in agents}) != len(agents):
        print(f"  Warning: {scen_name} has duplicate start locations")
    if len({(a[2], a[3]) for a in agents}) != len(agents):
        print(f"  Warning: {scen_name} has duplicate goal locations")


def write_scen(out_path, agents):
    with open(out_path, "w") as f:
        f.write("# start_x start_y start_z goal_x goal_y goal_z delay start_time\n")
        for sx, sy, gx, gy in agents:
            f.write(f"{sx} {sy} 0 {gx} {gy} 0 0 0\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("maps_dir", help="folder containing .map files")
    parser.add_argument("scen_dir", help="folder containing .scen files")
    parser.add_argument("output_dir", help="folder to write converted files into")
    args = parser.parse_args()

    for d in (args.maps_dir, args.scen_dir):
        if not os.path.isdir(d):
            sys.exit(f"Error: '{d}' is not a directory")

    os.makedirs(args.output_dir, exist_ok=True)

    # ---- Maps: one subfolder each ----
    map_data = {}  # stem -> (width, height, blocked)
    map_files = find_files(args.maps_dir, ".map")
    if not map_files:
        print(f"Warning: no .map files found in {args.maps_dir}")

    for path in map_files:
        stem = os.path.splitext(os.path.basename(path))[0]
        if stem in map_data:
            print(f"Warning: duplicate map name '{stem}' ({path}); overwriting")
        map_dir = os.path.join(args.output_dir, stem)
        os.makedirs(map_dir, exist_ok=True)
        out_path = os.path.join(map_dir, f"map_{stem}.txt")
        try:
            map_data[stem] = convert_map(path, out_path)
        except Exception as e:
            print(f"Error converting map {path}: {e}")
            continue
        print(f"Map  {os.path.basename(path)} -> {out_path}")

    # ---- Scenarios: into the folder of their map ----
    scen_files = find_files(args.scen_dir, ".scen")
    if not scen_files:
        print(f"Warning: no .scen files found in {args.scen_dir}")

    counts = {stem: 0 for stem in map_data}
    written = set()
    skipped = 0

    for path in scen_files:
        scen_stem = os.path.splitext(os.path.basename(path))[0]
        try:
            declared_map, agents = parse_scen(path)
        except Exception as e:
            print(f"Error reading scenario {path}: {e}")
            skipped += 1
            continue

        # Decide which map this scenario belongs to
        if declared_map in map_data:
            map_stem = declared_map
        else:
            map_stem = match_map_by_filename(scen_stem, list(map_data))
            if map_stem is None:
                print(f"Warning: no map found for {os.path.basename(path)} "
                      f"(declares '{declared_map}'); skipping")
                skipped += 1
                continue
            print(f"  Note: {os.path.basename(path)} declares map '{declared_map}', "
                  f"matched by filename to '{map_stem}'")

        check_agents(os.path.basename(path), agents, map_data[map_stem])

        out_path = os.path.join(args.output_dir, map_stem, f"scen_{scen_stem}.txt")
        if out_path in written:
            print(f"Warning: duplicate scenario name '{scen_stem}' ({path}); overwriting")
        written.add(out_path)
        write_scen(out_path, agents)
        counts[map_stem] += 1
        print(f"Scen {os.path.basename(path)} ({len(agents)} agents) -> {out_path}")

    # ---- Summary ----
    for stem, n in counts.items():
        if n == 0:
            print(f"Warning: map '{stem}' has no scenarios")
    print(f"\nDone: {len(map_data)} maps, {len(written)} scenarios written, "
          f"{skipped} skipped -> {args.output_dir}")


if __name__ == "__main__":
    main()