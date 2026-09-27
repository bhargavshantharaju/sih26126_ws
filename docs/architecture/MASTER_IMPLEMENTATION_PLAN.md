# SIH26126 — Master Implementation Plan

## Project

**Problem:** SIH26126 — Vision Based Autonomous Navigation for Unmanned
Ground Vehicle for Outdoor Environment

**Sponsor:** Bharat Electronics Limited (BEL)

**Stack:** ROS 2 Jazzy, Gazebo Harmonic, Ubuntu 24.04/WSL2

## Engineering Baseline

Known-good Git checkpoint:

`9a742f5`
`feat: establish verified autonomous navigation baseline`

The verified baseline contains:

- Gazebo UGV
- ROS-Gazebo bridge
- RGB camera
- camera_info
- IMU
- wheel/differential-drive odometry
- cmd_vel motion
- perception baseline
- traversability baseline
- read-only dashboard
- one-command full-stack launch

The baseline motion validation demonstrated approximately 2.095 m
forward displacement from a 0.5 m/s command for approximately 4 seconds,
followed by a correct stop.

The baseline remains the rollback point for all subsequent work.

---

## Architecture Direction

Target architecture:

Sensors
→ Sensor Contract
→ Perception + Geometry
→ Traversability Belief
→ State Estimation
→ Local World Model
→ Planning
→ Control Candidate
→ Safety Supervisor
→ Actuation

Every major subsystem should expose:

- value
- confidence/health
- timestamp
- failure state

Unknown terrain must never silently become FREE.

Safety owns final command authority.

---

# PHASE STATUS

| Phase | Objective | Status |
|---|---|---|
| P0 | Baseline stabilization | VERIFIED |
| P1 | Sensor trust / contracts | IN PROGRESS |
| P2 | Geometry / depth reasoning | NEXT |
| P3 | Traversability fusion | PLANNED |
| P4 | Wheel + IMU state estimation | PLANNED |
| P5 | Visual odometry correction | PLANNED |
| P6 | Local world/costmap | PLANNED |
| P7 | Global/mid-horizon planning | PLANNED |
| P8 | Local planning | PLANNED |
| P9 | Controller | PLANNED |
| P10 | Safety supervisor | PLANNED |
| P11 | Mission FSM / recovery | PLANNED |
| P12 | Active perception | PLANNED |
| P13 | Explainability / dashboard integration | PLANNED |
| P14 | Scenario benchmark / fault injection | PLANNED |
| P15 | Final autonomous mission integration | PLANNED |

---

# P1 — SENSOR TRUST FOUNDATION

## P1.1 Sensor Contract

Create:

`sih26126_msgs/msg/SensorHealth.msg`

Create:

`sih26126_sensor_contract`

The contract monitors:

- camera
- depth cloud
- IMU
- odometry

Each health output contains:

- sensor name
- data validity
- stale state
- overall health
- valid ratio
- age
- RGB/depth synchronization offset
- sample count
- machine-readable reason

Health topics:

- `/sensor_health/camera`
- `/sensor_health/depth`
- `/sensor_health/imu`
- `/sensor_health/odom`

## P1.2 Depth Quality Metric

The permanent runtime metric is:

`/geometry/valid_point_ratio`

The metric is computed over the navigation-relevant depth region,
not blindly over the whole image.

Depth acceptance must account for:

- finite XYZ
- minimum range
- maximum useful navigation range
- navigation ROI

The current observed cloud has approximately 47.3% finite XYZ over
the whole image, while the lower navigation region is substantially
more valid. Therefore whole-frame finite ratio must not be treated as
the final health criterion.

## P1.3 Timestamp Alignment

Monitor the difference between:

`/camera`

and

`/depth_camera/points`

Publish the measured synchronization offset through the depth health
contract.

## P1.4 Perception Latency

Measure image timestamp to downstream perception output timestamp.

The metric will later influence autonomy speed and health.

## P1 Acceptance

- custom message builds
- sensor-contract node runs
- health topics publish
- depth ratio publishes
- stale state is detectable
- invalid depth is detectable
- RGB/depth sync offset is measurable
- unit tests pass
- no raw source is modified merely to fake healthy status

---

# P2 — GEOMETRY

Implement:

- finite XYZ filtering
- valid range filtering
- ground-plane estimation
- surface normals
- local slope
- local roughness
- obstacle height
- geometry confidence

Outputs should support:

- traversability
- obstacle representation
- planner cost
- safety

Acceptance:

- known flat surface produces near-zero slope
- synthetic ramp produces measured slope
- obstacle produces measurable height/roughness change
- invalid points never become geometry

---

# P3 — TRAVERSABILITY

Replace the current camera-only heuristic as the authoritative
decision system.

Target representation:

FREE
CAUTION
BLOCKED
UNKNOWN

Signals:

- RGB appearance
- geometry
- surface slope
- roughness
- obstacle height
- semantic classification
- sensor reliability

Add:

- confidence
- temporal consistency
- sensor disagreement
- bounded terrain memory

Critical metric:

false-traversable rate

Safety rule:

UNKNOWN != FREE

---

# P4 — TRUSTED STATE ESTIMATION

Primary target:

wheel odometry + IMU

Use:

`robot_localization`

Produce:

- fused odometry
- covariance
- velocity
- health/confidence

Keep visual odometry as a correction source rather than the sole
authority.

Acceptance:

- square-path test
- Gazebo ground-truth comparison
- ATE
- RPE
- drift measurement

---

# P5 — VISUAL/RGB-D CORRECTION

The current custom RGB-D VO remains experimental.

Current known facts:

- unit tests pass
- RGB-D point cloud exists
- real motion test did not recover motion
- localization process previously crashed during runtime
- therefore it is not trusted

Only promote VO after:

- stable lifecycle
- usable correspondences
- accepted transforms
- drift benchmark
- rejection diagnostics
- comparison against Gazebo ground truth

---

# P6 — LOCAL WORLD MODEL

Use a bounded rolling local map.

Each cell should represent:

- cost
- confidence
- age
- state

Do not build a persistent global map until localization can support it.

TF target:

`map -> odom -> base_link -> sensor frames`

Do not publish a fake map transform.

---

# P7 — GLOBAL / MID-HORIZON PLANNING

Use a terrain-aware graph search over the available known region.

Start with A*.

Upgrade only when required by demonstrated limitations.

Planner must answer:

- where can I go?
- what does it cost?
- is the route sufficiently confident?
- is the goal reachable?

---

# P8 — LOCAL PLANNING

Build a real-time local planner consuming:

- local cost
- confidence
- localization
- vehicle limits
- global waypoints

UNKNOWN should be strongly penalized.

BLOCKED should be infeasible.

Local planning must support replanning.

---

# P9 — CONTROL

Keep control simple.

Diff-drive velocity/heading tracking is sufficient initially.

Do not duplicate complex optimization inside the controller if the
planner already reasons about terrain.

---

# P10 — SAFETY SUPERVISOR

Safety becomes the sole final command authority.

Target flow:

planner/controller candidate
→ safety validation
→ `/cmd_vel_safe`
→ vehicle

Safety checks:

- command bounds
- stale command
- camera health
- depth health
- IMU health
- localization confidence
- planner health
- obstacle clearance
- terrain confidence
- emergency stop

Failure behavior:

SAFE STOP unless an explicitly validated degraded mode exists.

---

# P11 — MISSION FSM / RECOVERY

Explicit states:

INIT
NOMINAL
DEGRADED
RECOVERY
SAFE_STOP

Recovery library:

1. re-observe / rotate
2. back-up and replan
3. lateral probe
4. human intervention

Every transition should generate a structured behavior event.

---

# P12 — ACTIVE PERCEPTION

When the route is uncertain:

do not blindly drive

and do not immediately freeze permanently.

Use a controlled probing maneuver to obtain a better viewpoint.

Potential behavior:

UNKNOWN region
→ slow
→ probe viewpoint
→ new observations
→ update belief
→ replan

---

# P13 — EXPLAINABILITY

Dashboard should explain:

- current mission state
- localization confidence
- traversability confidence
- sensor health
- current path
- reason for slowing
- reason for stopping
- reason for replanning
- recovery event

Behavior changes should have causal attribution.

---

# P14 — BENCHMARK / FAULT INJECTION

Minimum reproducible scenarios:

1. flat ground
2. static obstacles
3. narrow passage
4. uneven/slope
5. unknown terrain
6. low-texture scene
7. moving obstacle
8. depth degradation
9. camera degradation
10. localization degradation

Metrics:

Localization:
- ATE
- RPE
- drift

Perception:
- precision
- recall
- IoU

Traversability:
- IoU
- false-traversable rate
- false-blocked rate

Planning:
- success rate
- path length
- clearance
- replanning latency

Safety:
- safe-stop response time
- incorrect command rejection
- unsafe-command count

System:
- FPS
- latency
- CPU
- GPU
- memory

---

# DIFFERENTIATION TRACK

The following advanced capabilities should be added only after the
core stack is measurable:

- per-cell uncertainty propagation
- sensor disagreement score
- confidence-aware speed envelope
- terrain memory with confidence decay
- temporal traversability hysteresis
- online sensor reliability
- active perception
- structured recovery behaviors
- causal behavior event logging
- compute-aware perception
- perception latency watchdog
- slope-aware kinematic feasibility
- occluded-region risk inflation

These are engineering differentiators, not decorative features.

---

# NON-NEGOTIABLE RULES

1. Never break the verified baseline unnecessarily.
2. Never call a subsystem verified just because its node is alive.
3. Never let UNKNOWN silently become FREE.
4. Never allow raw invalid depth to become trusted geometry.
5. Never make RGB-D VO the sole localization authority without evidence.
6. Never allow the planner to bypass the safety supervisor.
7. Every major feature requires an acceptance test.
8. Simulation ground truth must be used for quantitative validation.
9. Do not add ML without a measured problem that it solves.
10. Prefer incremental changes with rollback points.

---

# CURRENT NEXT ACTIONS

1. Build `SensorHealth.msg`.
2. Build `sih26126_sensor_contract`.
3. Publish per-sensor health.
4. Publish permanent `/geometry/valid_point_ratio`.
5. Add tests for finite-depth handling.
6. Integrate the contract node into bringup only after standalone validation.
7. Then begin P2 geometry.

