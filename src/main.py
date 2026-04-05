"""
main.py — Async entry point for the ThymiAir navigation system.

Connects to the Thymio via tdmclient, captures an initial snapshot to plan the
global path, then drives the FSM: turn → go → re-plan on obstacle until goal reached.

Usage:
    python main.py
"""

import asyncio
import cv2
import numpy as np
from tdmclient import ClientAsync

import vision
import navigation
import kalman
import motion

# ---- camera setup ----------------------------------------------------------
CAMERA_INDEX = 0
T_S          = 0.1   # control loop period [s]
# ---------------------------------------------------------------------------


async def run(node, client, vid):
    """Main navigation loop: plan global path, execute turn-then-go FSM."""

    # Initial snapshot for static planning
    ret, frame = vid.read()
    snap_HSV   = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    poly_obst = vision.obstacle_detection(snap_HSV)
    cent_goal = vision.goal_detection(snap_HSV)

    flag, angle, cent_thym, _ = vision.find_thymio(snap_HSV)
    if not flag or cent_goal is None:
        print("Could not detect Thymio or goal in initial frame. Aborting.")
        return

    # Global path planning
    path_pos = navigation.path_construction(frame, poly_obst, cent_thym, cent_goal)

    # EKF initialisation
    x_est = np.array([[cent_thym[0]], [cent_thym[1]], [angle]])
    P_est = np.eye(3) * 100

    cam_flag = flag
    camera   = [cent_thym[0], cent_thym[1], angle]

    # Expose tdmclient handles to motion module (module-level globals)
    motion.node   = node
    motion.client = client

    # FSM: for each waypoint, turn then go
    for waypoint in path_pos[1:]:
        corner_pos = np.array(waypoint)
        thymio_pos = np.array([float(x_est[0]), float(x_est[1])])

        delta_angle, anticlockwise, thymio_orientation = motion.compute_angle(
            float(x_est[2]), corner_pos, thymio_pos
        )

        x_est, P_est = await motion.turn(
            delta_angle, anticlockwise, thymio_orientation,
            x_est, P_est, camera, cam_flag, T_S, vid
        )

        x_est, P_est, hit_obstacle = await motion.go_to_position(
            corner_pos, x_est, P_est, camera, cam_flag, T_S, vid
        )

        if hit_obstacle:
            print("Obstacle encountered — re-planning from current position.")
            ret, frame = vid.read()
            snap_HSV   = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            poly_obst  = vision.obstacle_detection(snap_HSV)
            current_pos = [int(x_est[0]), int(x_est[1])]
            path_pos   = navigation.path_construction(frame, poly_obst, current_pos, cent_goal)

    print("Goal reached.")
    motion.node.send_set_variables(motion.motors(0, 0))


async def main():
    vid = cv2.VideoCapture(CAMERA_INDEX)
    if not vid.isOpened():
        raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}")

    async with ClientAsync() as client:
        async with client.lock() as node:
            await run(node, client, vid)

    vid.release()


if __name__ == "__main__":
    asyncio.run(main())
