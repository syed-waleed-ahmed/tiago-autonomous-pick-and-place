# Challenges and Lessons Learned

What actually went wrong during development, and what we changed in response. Ordered by
task; the cross-cutting lessons are at the end.

---

## Task 1 — Map generation

### Odometry topic mismatch

**Problem.** Nav2 subscribes to `/odom`, but TIAGo publishes wheel odometry on
`/mobile_base_controller/odom`. Nothing crashed and no error appeared — the robot simply
never localised. The same mismatch exists for `/scan_raw` → `/scan` and for the velocity
command topic.

**Response.** Bridged all three with `topic_tools relay` processes in the launch files
rather than patching third-party configuration. Forking the vendored packages would have
worked too, but it would have made the workspace impossible to update cleanly.

### Odometry drift and laser noise

**Problem.** Drift smears walls in the occupancy grid and can leave duplicated geometry —
two parallel walls where there is one.

**Response.** Relied on SLAM Toolbox's pose-graph loop closure rather than trying to tune
it away, and ensured the exploration pattern revisited areas so the graph had closures to
optimise against. Loop closure, not parameter tuning, is what produces a clean map.

### Exploration stalling in doorways

**Problem.** Tight passages plus costmap inflation occasionally left no admissible local
trajectory, and the robot would sit still rather than route around.

**Response.** Balanced the inflation radius against the actual robot footprint, and let
Nav2's recovery behaviours clear the costmap and re-plan instead of adding our own
special case.

### Arm posture corrupting the map

**Problem.** TIAGo's default arm pose intrudes into the base laser's field of view,
painting phantom obstacles into the grid and making frontier selection thrash.

**Response.** The launch file folds the arm to `ARM_FOLD` at t = 20 s, before exploration
begins.

---

## Task 2 — Localization and discovery

### Global localization from a random pose

**Problem.** AMCL starts fully uncertain, and symmetric corridors cause perceptual
aliasing that no filter can resolve from a single viewpoint — the scan genuinely looks
the same from several poses, so the information is not there to be extracted.

**Response.** Commanded exploratory motion during `LOCALIZE` — a minimum of 2.5 rotations
plus translation nudges — and gated the transition out of the state on a measured
covariance threshold rather than on elapsed time. A visually tight particle cloud can
still be wrong; the numeric gate is what makes the run repeatable.

### Planner failure and goal aborts

**Problem.** The global planner intermittently failed to produce a valid plan
(`GridBased: failed to create plan with tolerance 0.50`) and goals were cancelled
mid-execution.

**Response.** Treated aborts as expected events rather than fatal ones: Nav2 recovery
clears the costmap and re-plans, and the state machine retries. Runs that look hesitant
in the recordings are recovery behaviours working as designed.

### Detecting markers off the direct path

**Problem.** A marker mounted on a side surface is invisible to a robot that only looks
straight ahead while driving, so a route that passes within a metre of a station can
still miss it.

**Response.** Added active head scanning at each patrol viewpoint — a full base spin,
then a head pan left and right — trading a little search time for a much higher
detection probability. Perception coverage turned out to be a motion-planning problem:
where the robot *looks* matters as much as where it drives.

---

## Task 3 — Pick and place

### Reachability at surface height

**Problem.** The pick surface sits where the 7-DoF arm has a narrow band of valid
configurations. Small pose errors made targets unreachable outright rather than merely
awkward.

**Response.** Used the lifting torso to bring the arm into a favourable working height
(0.20 m for manipulation, 0.10 m for transit) before the approach, instead of reaching
from a fixed torso position. The extra degree of freedom solved a problem the arm alone
could not.

### Detecting a 7 cm marker

**Problem.** Cube markers are a third the size of the station markers, so pose estimates
degrade quickly with distance and viewing angle. Reusing the 25 cm detector produced
poses good enough to navigate toward but not to grasp with.

**Response.** Ran a **second** `aruco_ros` instance configured at `marker_size:=0.07` on
its own topic, approached closely, and tilted the head to −0.55 rad so the marker fills
more of the image before the arm commits. Perception accuracy is task-relative: the
tolerance that satisfies navigation is not the tolerance that satisfies grasping.

### Simulation throughput

**Problem.** A real-time factor of roughly 0.4–0.7 meant every end-to-end test took
several minutes, and manipulation tuning needs many iterations.

**Response.** Tested individual states against an already-running pipeline instead of
re-running the whole mission for each change. Iteration speed, not ideas, became the
limiting factor on the manipulation work.

### Grasp reliability

**Problem.** A naive "attach the cube when close enough" approach would satisfy the
simulator but not the grading criteria, which require visible gripper actuation and
reasonable alignment.

**Response.** Sequenced the grasp explicitly — open, reach, descend, close, *then*
attach — so the behaviour is both physically sound and visibly correct. The cube is never
silently teleported into the gripper.

---

## Cross-cutting lessons

**Plumbing before algorithms.** Most early failures across all three tasks were transform
trees, topic names and frame conventions — not SLAM, not AMCL, not the arm. Verifying the
TF tree and topic graph before touching any algorithm parameter would have saved days.

**Measure, do not wait.** Every transition gated on a timeout became flaky; every one
gated on a measured quantity became repeatable. This shaped all three state machines, and
the AMCL covariance gate is the clearest example.

**Design for failure, not the happy path.** Nav2's recovery behaviours are the reason the
runs finish at all. Treating aborts, re-planning and retries as normal events is what
makes autonomy survive contact with a real simulator.

**Modularity paid for itself.** Because each task persisted a clean artefact — a map,
then a marker file — Task 3 inherited two-thirds of its pipeline already tested. The
[artefact chain](ARCHITECTURE.md#the-artefact-chain) was a design decision, not an
accident.

**A self-terminating explorer beats a map that merely looks complete.** `explore_lite`
stopping because it ran out of reachable frontiers is a far stronger completeness claim
than a timer expiring or a map that looks full to the eye.

**Know what you did not solve.** The grasp is closed-loop on perception; the release is
not. Being precise about that boundary is more useful than claiming the whole pipeline is
adaptive — see [Known limitations](ARCHITECTURE.md#7-known-limitations).
