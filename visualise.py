import viser
import numpy as np
import time
import json
import argparse
import colorsys

# Global state for the playback loop
is_playing = False

# --- HELPER FUNCTIONS ---
def generate_color(index, total):
    """Generates a distinct color for each drone based on its index."""
    hue = index / max(1, total)
    rgb = colorsys.hsv_to_rgb(hue, 0.8, 0.9)
    return tuple(int(c * 255) for c in rgb)

def load_map(filepath):
    """Parses the C++ map.txt file to get dimensions and obstacles."""
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    dims = (10, 10, 5) # fallback defaults
    obstacles = []
    dims_read = False
    
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) == 3:
            if not dims_read:
                dims = (int(parts[0]), int(parts[1]), int(parts[2]))
                dims_read = True
            else:
                obstacles.append((int(parts[0]), int(parts[1]), int(parts[2])))
                
    return dims, obstacles

# --- MAIN ---
def main():
    global is_playing

    # 1. Parse Command Line Arguments
    parser = argparse.ArgumentParser(description="Visualize MAPF paths.")
    parser.add_argument("-m", "--map", type=str, required=True, help="Path to map.txt")
    parser.add_argument("-p", "--paths", type=str, required=True, help="Path to output_paths.json")
    args = parser.parse_args()

    # 2. Load Data Dynamically
    dims, obstacles = load_map(args.map)
    w, h, d = dims

    with open(args.paths, 'r') as f:
        raw_agents = json.load(f)

    # Convert JSON lists to numpy arrays and assign colors
    agents_data = []
    for i, agent in enumerate(raw_agents):
        agents_data.append({
            "name": agent["name"],
            "color": generate_color(i, len(raw_agents)),
            "path": np.array(agent["path"], dtype=float)
        })

    # 3. Initialize Server
    server = viser.ViserServer()
    server.gui.configure_theme(
        dark_mode=True,
        brand_color=(40, 150, 255),
        control_layout="floating"
    )

    # 4. Add the Base Grid Floor (Dynamically sized and centered!)
    server.scene.add_grid(
        name="/environment/floor",
        width=w,
        height=h,
        position=((w-1)/2, (h-1)/2, -0.25), # Center the grid under the coordinates
        cell_color=(80, 80, 80),
        section_color=(120, 120, 120)
    )

    # Render map obstacles as semi-transparent boxes
    for obs in obstacles:
        server.scene.add_box(
            name=f"/environment/obstacles/obs_{obs[0]}_{obs[1]}_{obs[2]}",
            position=(obs[0], obs[1], obs[2]),
            dimensions=(1.0, 1.0, 1.0),
            color=(100, 100, 100),
            opacity=0.3 # Transparent so you can see drones behind them
        )

    # 5. Render Agents, Rigid Paths, and Landing Pads
    drone_handles = []
    for agent in agents_data:
        path = agent["path"]
        goal_pos = path[-1]

        # Ground pad for landing
        server.scene.add_box(
            name=f"/environment/pads/{agent['name']}_pad",
            position=(goal_pos[0], goal_pos[1], -0.15),
            dimensions=(1.0, 1.0, 0.1),
            color=agent["color"],
            wireframe=True
        )

        server.scene.add_spline_catmull_rom(
            name=f"/paths/{agent['name']}_trajectory",
            positions=path,
            line_width=3.0,
            color=agent["color"]
        )

        # Drone mesh
        drone = server.scene.add_icosphere(
            name=f"/agents/{agent['name']}",
            position=path[0],
            radius=0.25,
            color=agent["color"]
        )
        drone_handles.append((drone, path))

    # 6. Build the GUI Control Panel
    max_time_steps = max(len(a["path"]) for a in agents_data) - 1 if agents_data else 0

    with server.gui.add_folder("Flight Controls"):
        play_btn = server.gui.add_button("▶ Play")
        pause_btn = server.gui.add_button("⏸ Pause")
        time_slider = server.gui.add_slider(
            "Descent Timeline", min=0, max=max_time_steps, step=0.01, initial_value=0
        )
        status_text = server.gui.add_markdown("**Status:** Paused | **Progress:** 0%")

    # 7. GUI Callbacks
    @play_btn.on_click
    def _(_):
        global is_playing
        is_playing = True

    @pause_btn.on_click
    def _(_):
        global is_playing
        is_playing = False
        status_text.content = f"**Status:** Paused | **Progress:** {int((time_slider.value / max_time_steps) * 100)}%"

    # 8. Interpolation Logic (Lerp)
    @time_slider.on_update
    def _(_):
        t = time_slider.value
        idx, frac = int(t), t - int(t)

        for drone, path in drone_handles:
            if idx >= len(path) - 1:
                drone.position = path[-1]
            else:
                drone.position = (1.0 - frac) * path[idx] + frac * path[idx + 1]

        if not is_playing:
            status_text.content = f"**Status:** Paused | **Progress:** {int((t / max_time_steps) * 100)}%"

    # 9. Main Application Loop
    play_speed = 0.03
    while True:
        if is_playing:
            new_time = time_slider.value + play_speed
            if new_time > max_time_steps:
                new_time = 0.0  # Loop playback

            time_slider.value = new_time
            status_text.content = f"**Status:** Descending... | **Progress:** {int((new_time / max_time_steps) * 100)}%"

        time.sleep(0.05)


if __name__ == "__main__":
    main()