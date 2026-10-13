import os
import itertools
import argparse
from scenario_generator import generate_scenario

def generate_test_suite(suite_name="hive_out", parent_dir="tests"):
    # Create the nested directory path: e.g., tests/grid_search_base
    output_dir = os.path.join(parent_dir, suite_name)
    os.makedirs(output_dir, exist_ok=True)
    
    # 1. Define the grid search space
    dimensions = [
        (3, 3, 3),(4, 4, 4), (5,5,5), (6, 6, 6), (8, 8, 8), 
        (10, 10, 10), (12, 12, 12)
    ]
    agent_counts = [10, 50, 100, 200, 500]
    configs = ["any", "faces", "vent"]
    staggers = [1, 2, 3, 4, 6, 8, 10]
    
    # Fixed parameters based on your request
    fixed_params = {
        "spacing": 1,
        "chimney_height": 0,
        "ratio": 0.0,
        "delay": 5,
        "dynamic_prob_in": 0.0,
        "dynamic_prob_out": 1.0,
        "seed": 42
    }

    print(f"Generating test suite in '{output_dir}/'...")
    
    success_count = 0
    skipped_count = 0

    # 2. Iterate through all combinations
    for (w, h, d), agents, config, stagger in itertools.product(dimensions, agent_counts, configs, staggers):
        
        # Create unique filenames for this configuration
        base_name = f"w{w}_h{h}_d{d}_a{agents}_cfg-{config}_stag{stagger}"
        map_file = os.path.join(output_dir, f"map_{base_name}.txt")
        scen_file = os.path.join(output_dir, f"scen_{base_name}.txt")

        try:
            generate_scenario(
                w=w, h=h, d=d,
                num_agents=agents,
                map_file=map_file,
                scen_file=scen_file,
                config=config,
                stagger=stagger,
                **fixed_params
            )
            success_count += 1
            
        except ValueError as e:
            # Catches actual map limits like "No valid perimeter space" on tiny grids
            skipped_count += 1

    print("-" * 30)
    print(f"Test Suite '{suite_name}' Generation Complete!")
    print(f"Successfully generated: {success_count} scenarios.")
    print(f"Skipped (due to map generation limits): {skipped_count}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a MAPF test suite in a named subfolder")
    parser.add_argument("--name", type=str, default="run_001", help="Name of the subfolder inside the tests directory")
    parser.add_argument("--dir", type=str, default="tests", help="Parent directory for all tests")
    args = parser.parse_args()
    
    generate_test_suite(suite_name=args.name, parent_dir=args.dir)