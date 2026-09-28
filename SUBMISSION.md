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
Our driver is a reactive "disparity extender". On every LiDAR scan (about 40 Hz)
it looks at the beams within +/-100 degrees of straight ahead. Wherever two
neighbouring beams differ by more than 0.3 m, there is the edge of an obstacle,
so we "widen" the nearer obstacle by 0.30 m (half the car's 0.31 m width plus a
margin) over the far side. The car then steers towards the farthest remaining
open point, preferring the one closest to straight ahead when several are about
equally far. Widening obstacles means the car never aims at a gap it cannot fit
through, and it also seals the 0.25-0.50 m gaps between cones, so the car never
threads a cone row. If something is directly beside the car on the side it wants
to turn towards, it drives straight until it is clear, which stops it clipping
inside corners.

Speed is chosen from the steering angle: 7.0 m/s when straight, down to 3.5 m/s
at full steering lock, and never more than 1.5 x the distance to whatever is
directly ahead.

We chose this approach because it needs only the LiDAR, is simple enough for the
whole team to understand, and is robust. We tuned it by changing one parameter
at a time and measuring the result (see "What we tried"). The driver also reads
only the newest scan (subscription queue depth 1): with a queue of 10, a
struggling machine made the car react to old scans and hit walls.

### What we tried
| Change | Best lap | Collisions | Kept? |
| --- | --- | --- | --- |
| Template driver | - | 11 (DQ) | - |
| v1, max_speed 4.0 | 23.360 | 0 | - |
| Queue depth 10 -> 1 | 23.375 | 0 (was 1) | yes |
| max_speed 5.0 | 19.967 | 0 | - |
| max_speed 6.0 | 17.799 | 0 | - |
| max_speed 7.0 | 16.818 | 0 | yes |
| max_speed 8.0 | 16.997 (slower) | 0 | no |
| min_speed 1.5 -> 2.5 | 16.308 | 0 | - |
| min_speed 3.5 | 16.252 | 0 | yes |
| brake_gain 1.5 -> 2.0 | - | 11 (DQ) | no |

max_speed 8.0 was slower than 7.0: the car arrived at corners faster and lost
more time there than it gained on the straights. brake_gain 2.0 let the car
brake too late and it hit walls repeatedly.

## How it uses the inputs
- LiDAR: the only input the driving logic uses. Disparity extension, target
  selection, the side check and the speed limit are all computed from `/scan`.
- Odometry: subscribed to (ground truth, `/ego_racecar/odom`), but not used by
  this version.
- Map: not used by this version.

## Results on our own machine
The runs committed in `results/submitted/`, summarised.

| Best lap | 10-lap total | Collisions | Runs attempted |
| --- | --- | --- | --- |
| 15.791 s | 160.137 s | 0 in every run | 3 (all COMPLETE) |

All three runs from one `./scripts/evaluate.sh --team fifth_gear --runs 3 --headless`:

| Run | Status | Laps | Collisions | Best lap | 10-lap total |
| --- | --- | --- | --- | --- | --- |
| 20260928T080212Z_r1 | COMPLETE | 10/10 | 0 | 16.247 | 162.877 |
| 20260928T080212Z_r2 | COMPLETE | 10/10 | 0 | 15.791 | 160.938 |
| 20260928T080212Z_r3 | COMPLETE | 10/10 | 0 | 15.797 | 160.137 |

### The machine that produced them
| CPU | GPU | RAM | OS | Typical real-time factor |
| --- | --- | --- | --- | --- |
| Intel Core i7-11800H @ 2.30 GHz | NVIDIA GeForce RTX 3050 Ti Laptop | 7.6 GiB visible inside WSL | Windows 11, WSL 2 (Ubuntu 24.04), Docker Desktop | usually 1.3-1.5 headless |

With RViz open, our real-time factor sometimes dropped below 0.5, and in one run
with the old queue depth of 10 the car crashed because of it. All results above
are from headless runs.

## Anything precomputed
Nothing. The driver decides everything from the latest scan.

## Dependencies we added
None.

## Third-party code and references
- Disparity extender method: Nathan Otterness, UNC Chapel Hill F1TENTH team (2019).
- Template driver from this repository (preprocessing, publishing, marker code).

## AI assistance
We used Claude (Anthropic) to explain the setup, explain the template, draft the
first version of the driver, and plan the tuning experiments. We ran every
experiment ourselves and chose the parameters from the measured results.

## Known issues
- It is reactive: it only sees what is in front of it, so it cannot plan the best
  line through a corner. A racing line computed from the map and followed with
  ground-truth odometry should be faster.
- If the car ever gets wedged against a wall, it has no reverse or recovery
  behaviour, so it would be disqualified.
- It is tuned for this circuit. A much faster or tighter track would need
  retuning of max_speed, min_speed and brake_gain.
