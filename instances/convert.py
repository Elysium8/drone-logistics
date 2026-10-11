import os
import json
import random

def convert_dataset(base_dir="warehouseSmall", output_dir="warehouseSmall_converted"):
    if not os.path.exists(base_dir):
        print(f"Error: Could not find input directory '{base_dir}'.")
        return

    os.makedirs(output_dir, exist_ok=True)

    json_files = [f for f in os.listdir(base_dir) if f.endswith('.json')]
    if not json_files:
        print(f"Error: No JSON files found in {base_dir}.")
        return

    # Cache parsed maps to avoid re-parsing for every JSON
    parsed_maps = {}

    for json_file in json_files:
        with open(os.path.join(base_dir, json_file), 'r') as f:
            config = json.load(f)
        
        team_size = config["teamSize"]
        map_rel_path = config.get("mapFile")
        
        if not map_rel_path:
            print(f"Error: JSON {json_file} does not contain 'mapFile' key.")
            continue
            
        map_input_path = os.path.join(base_dir, map_rel_path)
        map_output_name = os.path.basename(map_rel_path)
        map_output_path = os.path.join(output_dir, map_output_name)

        if map_input_path not in parsed_maps:
            width = 0
            height = 0
            map_data_start = 0
            
            with open(map_input_path, 'r') as f:
                map_lines = f.readlines()

            for i, line in enumerate(map_lines):
                if line.startswith("width"):
                    width = int(line.split()[1])
                elif line.startswith("height"):
                    height = int(line.split()[1])
                elif line.startswith("map"):
                    map_data_start = i + 1
                    break

            valid_locations = []
            
            with open(map_output_path, 'w') as f:
                f.write(f"# width height depth\n{width} {height} 1\n")
                f.write("# obstacles (O x y z)\n# pads (P x y z)\n")
                
                for y, line in enumerate(map_lines[map_data_start:]):
                    clean_line = line.strip()
                    for x, char in enumerate(clean_line):
                        if char == '@' or char == 'T': 
                            f.write(f"O {x} {y} 0\n")
                        else:
                            # Any non-obstacle character is a valid spawn/goal location
                            valid_locations.append((x, y))
            
            parsed_maps[map_input_path] = valid_locations
            print(f"Converted map saved to {map_output_path}")
            
        valid_locations = parsed_maps[map_input_path]
        
        if team_size > len(valid_locations):
            print(f"Warning: teamSize ({team_size}) exceeds valid map locations. Skipping {json_file}.")
            continue

        scen_output_name = json_file.replace('.json', '.scen')
        scen_output_path = os.path.join(output_dir, scen_output_name)

        # Randomly sample unique starts and unique goals
        starts = random.sample(valid_locations, team_size)
        goals = random.sample(valid_locations, team_size)

        with open(scen_output_path, 'w') as f:
            f.write("# start_x start_y start_z goal_x goal_y goal_z delay start_time\n")
            
            for i in range(team_size):
                sx, sy = starts[i]
                gx, gy = goals[i]
                f.write(f"{sx} {sy} 0 {gx} {gy} 0 0 0\n")

        print(f"Generated {team_size} random agents for {json_file} -> {scen_output_path}")

if __name__ == "__main__":
    convert_dataset()