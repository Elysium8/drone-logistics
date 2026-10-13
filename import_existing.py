import os
import shutil
import argparse
import glob

def import_and_normalize(map_file, scen_dir, suite_name, parent_dir="tests"):
    dest_dir = os.path.join(parent_dir, suite_name)
    os.makedirs(dest_dir, exist_ok=True)
    
    if not os.path.exists(map_file):
        print(f"Error: Map file '{map_file}' not found.")
        return
    if not os.path.exists(scen_dir):
        print(f"Error: Scenario directory '{scen_dir}' not found.")
        return

    # Find all scenario files in the directory
    scen_files = glob.glob(os.path.join(scen_dir, "*.scen"))
    
    if not scen_files:
        print(f"No .scen files found in {scen_dir}")
        return

    copied_count = 0
    
    for scen_path in scen_files:
        # Extract the base name (e.g., "warehouseSmall_100")
        base_name = os.path.splitext(os.path.basename(scen_path))[0]
        
        # Create normalized destination paths
        dest_map = os.path.join(dest_dir, f"map_{base_name}.txt")
        dest_scen = os.path.join(dest_dir, f"scen_{base_name}.txt")
        
        # Copy the scenario file
        shutil.copy2(scen_path, dest_scen)
        
        # Duplicate the map file to match this specific scenario
        shutil.copy2(map_file, dest_map)
        
        copied_count += 1
        print(f"Paired: {os.path.basename(dest_map)} & {os.path.basename(dest_scen)}")

    print("-" * 30)
    print(f"Import Complete! Added {copied_count} scenario pairs to '{dest_dir}/'.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Import and normalize existing map/scen pairs")
    parser.add_argument("--map", type=str, required=True, help="Path to the single .map file")
    parser.add_argument("--scen_dir", type=str, required=True, help="Directory containing the .scen files")
    parser.add_argument("--name", type=str, required=True, help="Name of the destination test suite folder")
    parser.add_argument("--dir", type=str, default="tests", help="Parent directory for the test suite")
    
    args = parser.parse_args()
    import_and_normalize(args.map, args.scen_dir, args.name, args.dir)