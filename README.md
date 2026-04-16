# ThymiAir

Vision-based autonomous navigation for a Thymio II robot — obstacle detection, visibility-graph path planning, and EKF state estimation, all running from a single overhead camera.

> EPFL MICRO-452 Mobile Robotics — Group 09, Fall 2022  
> Team: **Albian Salihu**, Marin Bonnassies, Louis Gounot, Alexander Stephan

| Python | OpenCV | NumPy | tdmclient | EKF | Dijkstra |
|--------|--------|-------|-----------|-----|----------|

---

## Result

The robot successfully navigated a cluttered arena from start to goal during the course demo, using only an overhead camera — no onboard sensors for localization. The full pipeline (vision → planning → EKF → motion control) ran end-to-end in real time.

---

## My Role

My primary contribution was the **navigation module**: obstacle inflation, visibility-graph construction, and Dijkstra shortest-path selection. I also contributed to the vision pipeline and the overall system integration.

---

## System Architecture

```
┌──────────────┐   HSV frame    ┌─────────────────┐
│  Overhead    │ ─────────────► │   vision.py     │ → obstacle polygons
│  Camera      │                │                 │ → goal centroid
└──────────────┘                │                 │ → Thymio pose + heading
                                └────────┬────────┘
                                         │
                                ┌────────▼────────┐
                                │ navigation.py   │ inflate obstacles
                                │                 │ visibility graph
                                │                 │ Dijkstra → waypoints
                                └────────┬────────┘
                 odometry                │
                ──────────► ┌────────────▼────────┐
                            │   kalman.py         │ EKF predict + correct
                ◄────────── │                     │ → x_est, P_est
                 camera     └────────┬────────────┘
                                     │
                            ┌────────▼────────┐
                            │   motion.py     │ turn-then-go FSM
                            │   main.py       │ async tdmclient loop
                            └─────────────────┘
```

### Modules

| File | Responsibility |
|---|---|
| `src/vision.py` | HSV colorspace detection pipeline (15+ iterations to handle arena lighting). Detects obstacles as 4-corner rectangles, goal as a triangle, and Thymio via two colored dots — heading derived from the big→small dot vector. |
| `src/navigation.py` | Inflates each obstacle outward by `THYMIO_SIZE = 140 px` (robot radius). Builds a line-of-sight visibility graph over all inflated corners, start, and goal. Runs Dijkstra for the shortest path. |
| `src/kalman.py` | Extended Kalman Filter with a linearized unicycle motion model. Fuses wheel odometry (predict step) with overhead camera pose (correct step). Runs predict-only when the camera is occluded. |
| `src/motion.py` | `turn()` and `go_to_position()` control functions, proximity-sensor reactive avoidance, async via `tdmclient`. |
| `src/main.py` | Entry point: camera setup, Thymio connection, FSM loop. |

### FSM

![FSM diagram](assets/fsm.drawio.png)

| State | Behaviour |
|---|---|
| **IDLE** | Motors stopped; waits for start signal. |
| **Global** | Captures snapshot, runs full vision + path planning pipeline, transitions to Local. |
| **Local** | Executes turn-then-go toward each waypoint; runs Kalman filter every iteration. |
| **Avoid** | Proximity sensors triggered — spins away from obstacle, then returns to Global for re-planning. |

---

## Vision Pipeline

The detection pipeline went through 15+ iterations to handle real-world arena lighting conditions, color bleeding between markers, and varying camera angles.

| Step | Image |
|---|---|
| Raw camera frame | ![Raw frame](assets/testimg2.jpg) |
| HSV conversion | ![HSV image](assets/vis.HSV_image.png) |
| Saturation binary mask | ![Binary mask](assets/vis.binary_image.png) |
| All detected markers | ![All markers](assets/vis.all_markers.png) |

---

## Navigation Pipeline

| Step | Image |
|---|---|
| Obstacles inflated by robot radius | ![Augmented obstacles](assets/augmented.jpg) |
| Visibility graph (red = all visible edges) | ![Visibility graph](assets/visibility.jpg) |
| Shortest path selected by Dijkstra (blue) | ![Planned path](assets/path.jpg) |

---

## Setup

**Requirements:** Python 3.8+, Thymio II with Thymio Device Manager running, overhead USB/IP camera.

```bash
pip install -r requirements.txt
```

```bash
cd src
python main.py
```

`CAMERA_INDEX` in `main.py` defaults to `0` — change it if your overhead camera is not the primary device.

> **Reproducibility note:** HSV thresholds, polygon size ranges, and robot size assumptions are tuned to the original course arena and lighting. They will need re-tuning for any different physical setup.

---

## Known Limitations

- **Jerky motion** — the turn-then-go FSM performs a full stop-and-rotate before each straight segment; a smooth curvature controller would improve this significantly.
- **Pixel-space only** — all coordinates are in image pixels with no metric calibration, so absolute distances are approximate.
- **Camera occlusion** — if the Thymio body blocks its own markers during a tight turn, the EKF coasts on odometry alone until the markers reappear.
