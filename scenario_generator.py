import random
import math
import argparse
import os

def generate_scenario(w, h, d, num_agents, map_file, scen_file, spacing=2):
    # --- 1. Generate Goals (Landing Pads on the Ground Grid) ---
    # Calculate how many rows and columns we need for a square-ish grid
    cols = math.ceil(math.sqrt(num_agents))
    rows = math.ceil(num_agents / cols)
    
    # Calculate physical size of the grid based on spacing
    grid_w = (cols - 1) * spacing + 1
    grid_h = (rows - 1) * spacing + 1
    
    # Center the grid on the floor
    if grid_w > w or grid_h > h:
        raise ValueError(f"Map is too small to fit the grid! Needs at least {grid_w}x{grid_h}.")
        
    start_x = (w - grid_w) // 2
    start_y = (h - grid_h) // 2
    
    goals = []
    for i in range(num_agents):
        r = i // cols
        c = i % cols
        pad_x = start_x + (c * spacing)
        pad_y = start_y + (r * spacing)
        goals.append((pad_x, pad_y, 0)) # z = 0 is the ground
        
    # Shuffle goals! If drone 0 starts on the left but wants pad 10 on the right, 
    # it forces the A* solver to handle complex path crossings.
    random.shuffle(goals)
    
    # --- 2. Generate Starts (In the Air, Perimeter Only) ---
    perimeter_xy = []
    for x in range(w):
        for y in range(h):
            if x == 0 or x == w - 1 or y == 0 or y == h - 1:
                perimeter_xy.append((x, y))
                
    possible_starts = []
    for xy in perimeter_xy:
        for z in range(2, d): # Start higher up in the air (z=2 to max depth)
            possible_starts.append((xy[0], xy[1], z))
            
    if len(possible_starts) < num_agents:
        raise ValueError("Not enough perimeter space to spawn all drones! Increase map dimensions.")
        
    # Randomly pick unique starting locations
    starts = random.sample(possible_starts, num_agents)
    
    # --- 3. Write Map File ---
    os.makedirs(os.path.dirname(map_file), exist_ok=True)
    with open(map_file, 'w') as f:
        f.write("# width height depth\n")
        f.write(f"{w} {h} {d}\n")
        # You can add random obstacle generation here later!
        
    # --- 4. Write Scenario File ---
    os.makedirs(os.path.dirname(scen_file), exist_ok=True)
    with open(scen_file, 'w') as f:
        f.write("# start_x start_y start_z goal_x goal_y goal_z\n")
        for i in range(num_agents):
            sx, sy, sz = starts[i]
            gx, gy, gz = goals[i]
            f.write(f"{sx} {sy} {sz} {gx} {gy} {gz}\n")
            
    print(f"Generated {num_agents}-drone scenario on {w}x{h}x{d} map.")
    print(f"Outputs: {map_file} and {scen_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate drone MAPF scenarios")
    parser.add_argument("-w", "--width", type=int, default=20)
    parser.add_argument("-H", "--height", type=int, default=20)
    parser.add_argument("-d", "--depth", type=int, default=10)
    parser.add_argument("-a", "--agents", type=int, default=16)
    parser.add_argument("-s", "--spacing", type=int, default=2, help="Space between pads")
    parser.add_argument("--map", type=str, default="data/gen_map.txt")
    parser.add_argument("--scen", type=str, default="data/gen_scen.txt")
    
    args = parser.parse_args()
    generate_scenario(args.width, args.height, args.depth, args.agents, args.map, args.scen, args.spacing)