import random
import math
import argparse
import os

def generate_scenario(w, h, d, num_agents, map_file, scen_file, spacing=2, chimney_height=2, ratio=1.0, delay = 5, config="any", stagger=0, seed=None):
    if seed is not None:
        random.seed(seed)
        
    # --- 1. Split Agents based on Ratio ---
    num_incoming = int(num_agents * ratio)
    num_outgoing = num_agents - num_incoming
    num_pads_needed = max(num_incoming, num_outgoing)

    # --- 2. Generate Ground Pads ---
    cols = math.ceil(math.sqrt(num_pads_needed))
    rows = math.ceil(num_pads_needed / cols)
    
    grid_w = (cols - 1) * spacing + 1
    grid_h = (rows - 1) * spacing + 1
    
    if grid_w > w or grid_h > h:
        raise ValueError(f"Map is too small to fit the grid! Needs at least {grid_w}x{grid_h}.")
        
    start_x = (w - grid_w) // 2
    start_y = (h - grid_h) // 2
    
    pads = []
    pad_xy_set = set() # Track just the X,Y coordinates of the pads for obstacle generation
    
    for i in range(num_pads_needed):
        r = i // cols
        c = i % cols
        pad_x = start_x + (c * spacing)
        pad_y = start_y + (r * spacing)
        pads.append((pad_x, pad_y, 0))
        pad_xy_set.add((pad_x, pad_y))
        
    random.shuffle(pads)
    
    # Assign the pads to the two groups
    incoming_goals = random.sample(pads, k=num_incoming)
    outgoing_starts = random.sample(pads, k=num_outgoing)
    
    # --- 3. Generate Obstacles (The "Chimneys") ---
    obstacles = []
    # Fill the map with obstacles up to the chimney height...
    for z in range(chimney_height):
        for x in range(w):
            for y in range(h):
                # ...unless the x,y coordinate is exactly over a landing pad
                if (x, y) not in pad_xy_set:
                    obstacles.append((x, y, z))

    # --- 4. Generate Perimeter Locations (In the Air) ---
    spawn_z_start = max(2, chimney_height) 
    
    if config == "any":
        # Relaxed: Anywhere on the perimeter
        possible_locs = [(x, y, z) for x in range(w) for y in range(h) for z in range(spawn_z_start, d) 
                         if x == 0 or x == w - 1 or y == 0 or y == h - 1]
        
        if len(possible_locs) < max(num_incoming, num_outgoing):
            raise ValueError("Not enough perimeter space! Increase map dimensions or decrease chimney height.")
            
        incoming_starts = random.sample(possible_locs, num_incoming)
        outgoing_goals = random.sample(possible_locs, num_outgoing)

    elif config == "faces":
        # Restrictive: Incoming on North face, Outgoing on South face
        north_face = [(x, 0, z) for x in range(w) for z in range(spawn_z_start, d)]
        south_face = [(x, h - 1, z) for x in range(w) for z in range(spawn_z_start, d)]
        
        if len(north_face) < num_incoming or len(south_face) < num_outgoing:
            raise ValueError("Not enough space on the faces for this agent count!")
            
        incoming_starts = random.sample(north_face, num_incoming)
        outgoing_goals = random.sample(south_face, num_outgoing)

    elif config == "vent":
        # Extreme Bottleneck: All outgoing share south vent, all incoming share north vent
        south_vent = (w // 2, h - 1, d - 1)
        north_vent = (w // 2, 0, d - 1)
        
        outgoing_goals = [south_vent for _ in range(num_outgoing)]
        incoming_starts = [north_vent for _ in range(num_incoming)]
        
    else:
        raise ValueError(f"Unknown configuration: {config}")
    
    # --- 5. Write Map File ---
    os.makedirs(os.path.dirname(map_file), exist_ok=True)
    with open(map_file, 'w') as f:
        f.write("# width height depth\n")
        f.write(f"{w} {h} {d}\n")
        
        f.write("# obstacles (x y z)\n")
        for obs in obstacles:
            f.write(f"{obs[0]} {obs[1]} {obs[2]}\n")
        
    # --- 6. Write Scenario File ---
    os.makedirs(os.path.dirname(scen_file), exist_ok=True)
    with open(scen_file, 'w') as f:
        # Added the 8th column: start_time
        f.write("# start_x start_y start_z goal_x goal_y goal_z delay start_time\n")
        
        # Write Incoming Agents (with hardcoded exception for "vent")
        incoming_stagger = 2 if config == "vent" else stagger
        for i in range(num_incoming):
            sx, sy, sz = incoming_starts[i]
            gx, gy, gz = incoming_goals[i]
            start_time = i * incoming_stagger
            f.write(f"{sx} {sy} {sz} {gx} {gy} {gz} {delay} {start_time}\n")
            
        # Write Outgoing Agents (uses the standard stagger)
        for i in range(num_outgoing):
            sx, sy, sz = outgoing_starts[i]
            gx, gy, gz = outgoing_goals[i]
            start_time = i * stagger
            f.write(f"{sx} {sy} {sz} {gx} {gy} {gz} 0 {start_time}\n")
            
    print(f"Generated {num_agents}-drone scenario ({num_incoming} in, {num_outgoing} out) on {w}x{h}x{d} map.")
    print(f"Chimney Height: {chimney_height} (Total Obstacles: {len(obstacles)})")
    print(f"Outputs: {map_file} and {scen_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate drone MAPF scenarios")
    parser.add_argument("-w", "--width", type=int, default=20)
    parser.add_argument("-H", "--height", type=int, default=20)
    parser.add_argument("-d", "--depth", type=int, default=10)
    parser.add_argument("-a", "--agents", type=int, default=16)
    parser.add_argument("-s", "--spacing", type=int, default=2, help="Space between pads")
    parser.add_argument("-c", "--chimneys", type=int, default=2, help="Height of the ground constraints")
    parser.add_argument("--map", type=str, default="data/gen_map.txt")
    parser.add_argument("--scen", type=str, default="data/gen_scen.txt")
    parser.add_argument("--ratio", type=float, default=1.0, help="Ratio of incoming drones (0.0 to 1.0)")
    parser.add_argument("--delay", type=int, default=5, help="Timesteps incoming drones occupy the pad before disappearing")
    parser.add_argument("--config", type=str, choices=["any", "faces", "vent"], default="any", help="Perimeter spawn setup")
    parser.add_argument("--stagger", type=int, default=0, help="Timesteps between drone spawns")
    parser.add_argument("--seed", type=int, default=72, help="Random seed for reproducibility")
    
    args = parser.parse_args()
    generate_scenario(args.width, args.height, args.depth, args.agents, args.map, args.scen, args.spacing, args.chimneys, args.ratio, args.delay, args.config, args.stagger, args.seed)