# Team Fifth Gear

## Members
| Name | Matric number | Email |
| --- | --- | --- |
| Rohit G Suresh | TODO | rohitgownahallisuresh@gmail.com |
| TODO | TODO | TODO |
| TODO | TODO | TODO |
| TODO | TODO | TODO |
| TODO | TODO | TODO |

## Approach
Our judged driver (`ros2 run team_driver driver`, file `racing_driver.py`) is a
path follower that plans ahead. At startup it loads a path around the track,
smooths it with a 5-point moving average, and measures its curvature at every
point from the circle through each point and its neighbours three points away.
It then plans the fastest speed at every point: in corners the speed is limited
by v = sqrt(a_lat / curvature), and two passes around the loop (one backwards,
one forwards) limit how hard the car has to brake before a corner and how fast
it can accelerate out of one. The driver logs the ideal lap time this plan
predicts, which let us tell whether the plan or the tracking was limiting us.

While driving, the car finds its position on the path from ground-truth
odometry, searching only just around its last position so it never jumps to a
nearby part of the infield. It then uses pure pursuit: it picks a target point
0.6 to 2.0 m ahead on the path (further at higher speed) and steers along the
circular arc that reaches it. It drives at the planned speed, capped by the
LiDAR if something is very close directly ahead.

For this version the path is the track centreline, not an optimised racing line.
Our first driver (v1, still in the package as `driver_v1`, file `driver.py`) was
a reactive disparity extender that used only the LiDAR. It was reliable but
could not plan ahead: its best 10-lap total was 160.14 s. The path follower
brought that down to 136.83 s.

### What we tried (v2, 3-lap runs unless noted)
| Change | Predicted lap | Best lap | Collisions | Kept? |
| --- | --- | --- | --- | --- |
| a_lat 5, v_max 8, a_acc 5, a_brake 5 | 20.06 | 21.46 | 0 | - |
| a_lat 7 | 17.74 | 19.21 | 0 | - |
| a_lat 9 | 16.17 | 17.89 | 0 | - |
| a_lat 11 | 15.01 | 16.96 | 0 | - |
| a_lat 13 | 14.11 | 16.16 | 0 | - |
| a_lat 15 | 13.38 | 15.57 | 0 | - |
| a_lat 17 | 12.77 | 15.11 | 0 | - |
| a_lat 19 | 12.25 | 14.74 | 0 | - |
| a_lat 21 (10 laps: 145.13 s) | 11.82 | 14.48 | 0 | yes |
| v_max 10 | 11.73 | 14.40 | 0 | yes |
| a_acc 8 | 11.47 | 14.26 | 0 | yes |
| a_brake 8 | 11.12 | 13.74 | 0 | yes |
| a_brake 11 | 10.93 | - | 11 (DQ) | no |

We stopped raising a_lat when each step gained less than 0.3 s. The best a_lat
(21 m/s^2) is far above the real tyre grip of the simulated car (about 10 m/s^2).
We think this is because pure pursuit cuts corners: the car drives a wider arc
than the centreline, so it corners less sharply than the plan assumes, and a_lat
ends up compensating for that rather than measuring grip. a_brake 11 made the car
arrive at an early corner too fast; it hit the wall before timing started and
stayed against it until it was disqualified.

### v1 tuning (for reference)
max_speed 4 -> 7 m/s (8 was slower), min_speed 1.5 -> 3.5 m/s, brake_gain 1.5
(2.0 was disqualified). Changing the scan queue depth from 10 to 1 fixed crashes
on a struggling machine, because the car had been reacting to old scans.

## How it uses the inputs
- LiDAR: an emergency speed cap only (speed <= 2.5 x the distance straight ahead).
- Odometry: ground truth from `/ego_racecar/odom`, for position, heading and speed.
  We did not build our own localisation.
- Map: we use `maps/icra26_centerline.csv`, which the organisers' `track_tool.py`
  generates from the map. We do not read the occupancy grid directly yet.

## Results on our own machine
The runs committed in `results/submitted/`, summarised.

| Best lap | 10-lap total | Collisions | Runs attempted |
| --- | --- | --- | --- |
| 13.637 s | 136.828 s | 0, 0, 1 | 3 (all COMPLETE) |

All three runs from one `./scripts/evaluate.sh --team fifth_gear --runs 3 --headless`:

| Run | Status | Laps | Collisions | Best lap | 10-lap total |
| --- | --- | --- | --- | --- | --- |
| 20260928T141724Z_r1 | COMPLETE | 10/10 | 0 | 13.637 | 136.967 |
| 20260928T141724Z_r2 | COMPLETE | 10/10 | 0 | 13.642 | 136.828 |
| 20260928T141724Z_r3 | COMPLETE | 10/10 | 1 | 13.640 | 137.093 |

The collision in run 3 carried no time penalty, so it happened on the out lap or
warm-up lap, before timing started.

### The machine that produced them
| CPU | GPU | RAM | OS | Typical real-time factor |
| --- | --- | --- | --- | --- |
| Intel Core i7-11800H @ 2.30 GHz | NVIDIA GeForce RTX 3050 Ti Laptop | 7.6 GiB visible inside WSL | Windows 11, WSL 2 (Ubuntu 24.04), Docker Desktop | usually 1.3-1.9 headless |

With RViz open, our real-time factor sometimes dropped below 0.5. All results
above are from headless runs.

## Anything precomputed
Nothing is shipped as data by us. At startup the driver reads the organisers'
centreline (`maps/icra26_centerline.csv`, regenerated with
`./scripts/track_tool.py centerline`), then smooths it, computes curvature and
plans the speed profile itself. The whole plan is rebuilt every time the driver
starts.

## Dependencies we added
None.

## Third-party code and references
- Pure pursuit: R. Craig Coulter, "Implementation of the Pure Pursuit Path
  Tracking Algorithm", CMU-RI-TR-92-01, Carnegie Mellon University, 1992.
- v1's disparity extender: Nathan Otterness, UNC Chapel Hill F1TENTH team (2019).
- Template driver from this repository (message handling and publishing code).
- The centreline in `maps/`, generated by the organisers' `track_tool.py`.

## AI assistance
We used Claude (Anthropic) to explain the setup and the template, draft both
drivers, and plan the tuning experiments. We ran every experiment ourselves and
chose every parameter from the measured results above.

## Known issues
- The path is the centreline, not a racing line. A line that runs wide into
  corners and clips the apex would be both shorter and faster.
- No recovery: if the car ever ends up against a wall it cannot reverse off it,
  so it would be disqualified (as happened with a_brake 11).
- One run touched a wall before timing started. It cost no time, but it counts
  towards the 10-collision limit.
- a_lat does not measure real grip (see above), so it would need retuning on a
  different track or path.
