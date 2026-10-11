import os
import json
import math

def convert_dataset(base_dir="warehouseSmall", output_dir="warehouseSmall_converted"):
    if not os.path.exists(base_dir):
        print(f"Error: Could not find input directory '{base_dir}'.")
        return

    # Create the output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # 1. Find a JSON file to determine the map file path
    json_files = [f for f in os.listdir(base_dir) if f.endswith('.json')]
    if not json_files:
        print(f"Error: No JSON files found in {base_dir}.")
        return

    with open(os.path.join(base_dir, json_files[0]), 'r') as f:
        first_config = json.load(f)
    
    map_rel_path = first_config.get("mapFile")
    if not map_rel_path:
        print("Error: JSON does not contain 'mapFile' key.")
        return

    map_input_path = os.path.join(base_dir, map_rel_path)
    map_output_name = os.path.basename(map_rel_path)
    map_output_path = os.path.join(output_dir, map_output_name)

    # 2. Parse the Moving AI map and convert it
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

    # Write the converted map
    with open(map_output_path, 'w') as f:
        f.write(f"# width height depth\n{width} {height} 1\n")
        f.write("# obstacles (O x y z)\n# pads (P x y z)\n")
        
        for y, line in enumerate(map_lines[map_data_start:]):
            clean_line = line.strip()
            for x, char in enumerate(clean_line):
                if char == '@':
                    f.write(f"O {x} {y} 0\n")

    print(f"Converted map saved to {map_output_path}")

    # 3. Process each JSON file to create individual .scen files
    for json_file in json_files:
        with open(os.path.join(base_dir, json_file), 'r') as f:
            config = json.load(f)
        
        team_size = config["teamSize"]
        task_rel_path = config["taskFile"]
        task_input_path = os.path.join(base_dir, task_rel_path)
        
        scen_output_name = json_file.replace('.json', '.scen')
        scen_output_path = os.path.join(output_dir, scen_output_name)

        with open(task_input_path, 'r') as f:
            task_lines = f.readlines()

        seen_starts = set()
        seen_goals = set()
        agents_loaded = 0

        with open(scen_output_path, 'w') as f:
            f.write("# start_x start_y start_z goal_x goal_y goal_z delay start_time\n")
            
            # Skip the first two lines (version header and total task count)
            for line in task_lines[2:]:
                clean_line = line.strip()
                if not clean_line:
                    continue
                
                parts = clean_line.split(',')
                if len(parts) != 2:
                    continue
                
                start_1d = int(parts[0])
                goal_1d = int(parts[1])

                # Filter duplicates based on unique starts and goals
                if start_1d in seen_starts or goal_1d in seen_goals:
                    continue

                seen_starts.add(start_1d)
                seen_goals.add(goal_1d)

                # Unflatten the 1D indices based on map width
                sx = start_1d % width
                sy = start_1d // width
                gx = goal_1d % width
                gy = goal_1d // width

                # Write custom format line with z=0, delay=0, start_time=0
                f.write(f"{sx} {sy} 0 {gx} {gy} 0 0 0\n")
                
                agents_loaded += 1
                if agents_loaded >= team_size:
                    break

        print(f"Converted {team_size} agents from {task_rel_path} -> {scen_output_path}")

if __name__ == "__main__":
    convert_dataset()