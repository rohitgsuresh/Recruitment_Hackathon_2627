# Team Fifth Gear

## Members

| Name | Matric number | Email |
| --- | --- | --- |
| Rohit Gownahalli Suresh | U2623211D | rohit023@e.ntu.edu.sg |
| Akash Vaidyanathan | U2621742A | akas0002@e.ntu.edu.sg |
| Sabitendra Telukula | U2621815K | sabitend001@e.ntu.edu.sg |
| Sarvesh Srinivasan | U2622152J | sarv0004@e.ntu.edu.sg |
| Jeevan Santhanam | U2623111G | jeevan006@e.ntu.edu.sg |

## Approach

Our judged executable is `ros2 run team_driver driver`, implemented in
`racing_driver.py`. It follows a smoothed track centreline using pure pursuit,
with a planned speed profile and bounded yaw-rate feedback.

At startup, the driver loads the organisers' `maps/icra26_centerline.csv` and
smooths it with a five-point circular moving average. It estimates signed
curvature from the circle through each point and its neighbours three points
away. The smoothed path has 392 points and is approximately 77.93 m long.

The initial speed limit at each point is the smaller of `v_max` and
`sqrt(a_lat / max(abs(curvature), 1e-6))`. Two backward sweeps around the closed
loop impose braking constraints, followed by two forward sweeps imposing
acceleration constraints. Repeating each sweep lets constraints propagate
across the start/end of the path. The logged predicted lap time assumes the
planned speeds are achieved along that path; it is not a physical lower bound
or a measured lap time.

During driving, ground-truth odometry provides position, heading, speed and
yaw rate. The driver initially checks the path direction against the car's
heading and reverses it if needed. It normally searches for the nearest path
point in a local window around its previous index, reducing the chance of
switching to a nearby section of the infield. Initial acquisition, or losing
the local path by more than 2 m, uses a global search.

For steering, pure pursuit selects a discrete target point ahead along the
path. Lookahead is `clip(0.6 + 0.15 * speed, 0.6, 2.0)` metres. The target is
transformed into the car's frame, and the controller calculates the circular
arc to it using a 0.3302 m wheelbase.

We then add bounded yaw-rate feedback. If speed is above 2 m/s and measured
yaw rate is finite, the controller calculates:

```text
requested_yaw_rate = speed * tan(pure_pursuit_steering) / wheelbase
correction = clip(0.0225 * (requested_yaw_rate - measured_yaw_rate), -0.05, 0.05)
steering = clip(pure_pursuit_steering + correction, -0.4189, 0.4189)
```

This uses a kinematic yaw-rate reference and adds a limited steering correction
when measured rotation differs from the requested rotation. The correction
is limited to about 2.86 degrees. We selected the gain and limit through
simulation experiments; they do not guarantee stable tracking on other paths.

The speed command uses the planned speed at the nearest path point, with no
forward index preview. LiDAR caps it at three times the minimum range within
five degrees of straight ahead. The final speed command has a 0.5 m/s floor,
so this cap is not an emergency-stop guarantee. Scan and odometry subscriptions
use queue depth 1 to reduce message backlog.

### Selected settings

| Setting | Value |
| --- | --- |
| Path | Smoothed organiser centreline |
| Maximum planned speed | 10.0 m/s |
| Lateral-acceleration planning parameter | 21.0 m/s² |
| Acceleration planning limit | 9.0 m/s² |
| Braking planning limit | 8.0 m/s² |
| Speed scale / preview | 1.0 / 0 points |
| Lookahead base / speed gain | 0.6 m / 0.15 s |
| Lookahead minimum / maximum | 0.6 m / 2.0 m |
| Pure-pursuit steering gain | 1.0 |
| Yaw-feedback gain / correction limit | 0.0225 / ±0.05 rad |
| LiDAR speed-cap gain | 3.0 s⁻¹ |
| Extra corner-cut limit, side check and tight-corner cap | Disabled |

The lateral-acceleration parameter is an empirical tuning value, not a measured
tyre-grip limit. The driven trajectory differs from the planned centreline,
and the speed planner does not model combined tyre forces, sideslip or tracking
dynamics. These differences limit the interpretation of its predicted times.

## Development and experiments

Our first driver, `driver_v1` in `driver.py`, was a LiDAR-only disparity
extender. Its recorded best ten-lap total was 160.14 s. Moving to a centreline
speed planner and pure pursuit reduced that total, and subsequent tuning
focused on repeatability as well as speed.

Selected three-run repeat batches are below. Every run in these batches
completed ten timed laps. Totals include referee penalties. These are different
configurations, not three attempts selected from one larger batch.

| Configuration | Ten-lap totals (s), runs 1 / 2 / 3 | Collisions, runs 1 / 2 / 3 | Decision |
| --- | --- | --- | --- |
| No yaw feedback, speed preview 0, acceleration 8 | 131.860 / 131.656 / 131.833 | 0 / 0 / 0 | Earlier baseline |
| Yaw gain 0.0225, correction cap 0.04, acceleration 8 | 128.274 / 128.055 / 127.917 | 0 / 0 / 0 | Earlier baseline |
| Yaw gain 0.025, correction cap 0.04, acceleration 8 | 127.776 / 128.050 / 137.897 | 0 / 0 / 3 | Not selected; run 3 had a timed collision |
| Yaw gain 0.0225, correction cap 0.04, acceleration 8.5 | 137.955 / 127.654 / 127.471 | 1 / 0 / 0 | Not selected; run 1 had a timed collision |
| Yaw gain 0.0225, correction cap 0.05, acceleration 8 | 127.807 / 127.823 / 127.820 | 0 / 0 / 0 | Kept wider correction cap |
| Same feedback, acceleration 8.5 | 127.436 / 127.270 / 127.318 | 0 / 0 / 0 | Faster clean intermediate version |
| Same feedback, acceleration 9 | 127.065 / 127.034 / 127.089 | 0 / 0 / 0 | Selected |

We recorded odometry, drive commands and LiDAR to inspect tracking. Replaying
the feedback calculation on recorded poses suggested that the 0.04 rad
correction bound was frequently active on a troublesome corner approach. This
motivated testing a 0.05 rad bound. Replay was diagnostic only; the complete
simulation runs determined whether to retain the change.

We also tested an experimental map-derived racing line through `driver_v21`.
It minimises bending with a path-length penalty inside estimated lateral
corridor bounds. Its earlier best reported clean lap was about 14.05 s. With
the newer yaw feedback, a screening run was disqualified during warm-up after
contact near (5.92, 7.68) and becoming stuck around (5.54, 7.38). Startup
reported 11 pinned points and 10 points outside its requested clearance.
This experiment is retained in the package but is not the judged executable.
We selected the centreline controller based on its measured speed and
repeatability rather than the racing-line planner's optimistic predictions.

## How it uses the inputs

- **LiDAR (`/scan`):** speed cap from the forward five-degree half-angle sector.
- **Odometry (`/ego_racecar/odom`):** simulator ground-truth position, heading,
  planar speed and angular velocity about the vertical axis. We did not build
  our own localisation.
- **Map-derived path:** the judged driver reads the organiser centreline CSV.
  It does not use the occupancy grid for runtime obstacle avoidance. The
  experimental `driver_v21` reads the map image and YAML for path generation.
- **Output:** `AckermannDriveStamped` on `/drive`, plus visualisation markers
  on `/driver/markers`.

## Results on our own machine

The selected configuration was screened once and then evaluated in one
three-run repeat batch using:

```bash
./scripts/evaluate.sh --team v20_yaw00225_cap005_acc9_repeat --driver-exec driver --laps 10 --runs 3 --headless
```

| Run | Status | Timed laps | Collisions | Best lap (s) | Ten-lap total (s) |
| --- | --- | --- | --- | --- | --- |
| 20261017T011624Z_r1 | COMPLETE | 10/10 | 0 | 12.680 | 127.065 |
| 20261017T011624Z_r2 | COMPLETE | 10/10 | 0 | 12.683 | 127.034 |
| 20261017T011624Z_r3 | COMPLETE | 10/10 | 0 | 12.695 | 127.089 |

The repeat-batch mean total was 127.063 s. The fastest repeat-batch lap and
best total came from different runs. The preceding screening run,
`20261017T010546Z`, completed with zero collisions, a 12.673 s best lap and
127.137 s total. Thus all four observed sessions for the selected
configuration completed without recorded collisions; this is not a guarantee
of collision-free judging runs.

Repeat-batch result filenames are
`results/submitted/v20_yaw00225_cap005_acc9_repeat__20261017T011624Z_r1.json`
through `_r3.json`. All three files were copied byte-for-byte from the referee
output without editing their contents. These figures are local evaluation
results, not official judging results.

### Machine

| CPU | GPU | RAM | OS | Typical real-time factor |
| --- | --- | --- | --- | --- |
| Intel Core i7-11800H @ 2.30 GHz | NVIDIA GeForce RTX 3050 Ti Laptop | 7.6 GiB visible inside WSL | Windows 11, WSL 2 (Ubuntu 24.04), Docker Desktop | Approximately 0.50x in the submitted batch |

The reported experiments used headless runs. The submitted runs recorded
real-time factors of 0.501, 0.497 and 0.497 respectively. Real-time factor varied
with machine load; referee lap times use the simulator's clock.

## Anything precomputed

We supply no separately optimised path or speed-profile data for the judged
driver. It reads the organiser centreline and rebuilds smoothing, curvature
and the speed profile at every startup. The experimental racing-line path is
also computed at startup and is not used by `driver`.

## Dependencies we added

No additional packages are declared in our team's `requirements.txt` or
`apt-packages.txt`. The source imports NumPy, SciPy, Pillow and PyYAML, along
with ROS 2 and its message packages, from the supplied environment. SciPy and
Pillow support the experimental map-based planner; these imports are currently
at module scope even when running the centreline driver.

## Third-party code and references

- Pure pursuit: R. Craig Coulter, "Implementation of the Pure Pursuit Path
  Tracking Algorithm", CMU-RI-TR-92-01, Carnegie Mellon University, 1992.
- v1 disparity extender: Nathan Otterness, UNC Chapel Hill F1TENTH team (2019).
- Repository template driver: message handling and publishing code.
- Organiser centreline in `maps/`, generated by `track_tool.py`.
- SciPy's L-BFGS-B optimiser and Euclidean distance transform are used by the
  experimental racing-line planner.

## AI assistance

We used Claude (Anthropic) to explain the setup and template, draft the initial
drivers, and help plan tuning experiments. We used ChatGPT/Codex (OpenAI) to
review code and logs, analyse recorded telemetry, propose and draft yaw-rate
feedback and parameter changes, and help revise this submission document.
The team ran the simulator experiments and evaluated the measured results to
select the submitted configuration. AI-generated suggestions and offline
calculations were not treated as substitutes for simulation testing.

## Known issues

- The judged path is a smoothed centreline, not a jointly optimised trajectory
  and speed profile. Pure pursuit and vehicle dynamics cause deviations from
  that path.
- No reversing recovery is implemented. Sustained wall contact can cause
  disqualification or a stuck timeout.
- The LiDAR cap covers a narrow forward sector and has a minimum speed floor;
  it does not ensure clearance for the whole vehicle or handle every obstacle.
- The planner's acceleration parameters are empirical. Changed paths or
  vehicle dynamics require retuning, and increasing individual parameters has
  caused collisions in experiments.
- The experimental racing-line planner does not reliably satisfy its requested
  clearance everywhere, and its controller settings have not achieved the
  selected centreline version's performance or reliability.
