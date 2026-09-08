import viser
import numpy as np
import time

# Global state for the playback loop
is_playing = False


def main():
    global is_playing

    # 1. Initialize Server and Force Dark Theme (UPDATED API)
    server = viser.ViserServer()
    server.gui.configure_theme(
        dark_mode=True,
        brand_color=(40, 150, 255),
        control_layout="floating"
    )

    # 2. Add the Base Grid Floor (UPDATED API)
    server.scene.add_grid(
        name="/environment/floor",
        width=20,
        height=20,
        position=(0, 0, -0.25),
        cell_color=(80, 80, 80),
        section_color=(120, 120, 120)
    )

    # 3. 26-Way Grid-Locked Trajectories
    agents_data = [
        {
            "name": "Drone_Alpha",
            "color": (255, 140, 0),
            "path": np.array(
                [[4, 4, 8], [3, 3, 7], [2, 2, 6], [1, 1, 5], [1, 1, 4], [0, 0, 3], [-1, -1, 2], [-1, -1, 1],
                 [-2, -2, 0]])
        },
        {
            "name": "Drone_Bravo",
            "color": (50, 200, 200),
            "path": np.array(
                [[-4, 4, 8], [-3, 3, 7], [-2, 2, 6], [-2, 1, 5], [-1, 0, 4], [-1, -1, 3], [0, -1, 2], [0, -2, 1],
                 [0, -2, 0]])
        },
        {
            "name": "Drone_Charlie",
            "color": (255, 50, 150),
            "path": np.array(
                [[0, -5, 8], [0, -4, 7], [1, -4, 6], [1, -3, 5], [1, -3, 4], [2, -3, 3], [2, -2, 2], [2, -2, 1],
                 [2, -2, 0]])
        },
        {
            "name": "Drone_Delta",
            "color": (50, 255, 50),
            "path": np.array(
                [[5, -4, 8], [4, -3, 7], [3, -2, 6], [2, -1, 5], [1, -1, 4], [0, 0, 3], [-1, 1, 2], [-1, 2, 1],
                 [-2, 2, 0]])
        },
        {
            "name": "Drone_Echo",
            "color": (150, 50, 255),
            "path": np.array(
                [[-5, -4, 8], [-4, -3, 7], [-3, -2, 6], [-3, -1, 5], [-2, 0, 4], [-1, 1, 3], [-1, 1, 2], [0, 2, 1],
                 [0, 2, 0]])
        },
        {
            "name": "Drone_Foxtrot",
            "color": (255, 255, 50),
            "path": np.array(
                [[0, 5, 8], [0, 4, 7], [1, 4, 6], [1, 3, 5], [1, 3, 4], [2, 3, 3], [2, 2, 2], [2, 2, 1], [2, 2, 0]])
        }
    ]

    # 4. Render Agents, Rigid Paths, and Landing Pads
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

        # Use the supported spline primitive instead of edges (UPDATED API)
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

    # 5. Build the GUI Control Panel
    max_time_steps = max(len(a["path"]) for a in agents_data) - 1

    with server.gui.add_folder("Flight Controls"):
        play_btn = server.gui.add_button("▶ Play")
        pause_btn = server.gui.add_button("⏸ Pause")
        time_slider = server.gui.add_slider(
            "Descent Timeline", min=0, max=max_time_steps, step=0.01, initial_value=0
        )
        status_text = server.gui.add_markdown("**Status:** Paused | **Progress:** 0%")

    # 6. GUI Callbacks
    @play_btn.on_click
    def _(_):
        global is_playing
        is_playing = True

    @pause_btn.on_click
    def _(_):
        global is_playing
        is_playing = False
        status_text.content = f"**Status:** Paused | **Progress:** {int((time_slider.value / max_time_steps) * 100)}%"

    # 7. Interpolation Logic (Lerp)
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

    # 8. Main Application Loop
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