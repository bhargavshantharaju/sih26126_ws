Vision-Based Autonomous Navigation for an Unmanned Ground Vehicle (UGV)

Smart India Hackathon 2026 — SIH26126

Sponsor: Bharat Electronics Limited (BEL)
Category: Software
Theme: Smart Automation / Robotics and Drones

A camera-primary autonomous navigation stack for an outdoor unmanned ground vehicle, developed around ROS 2 Jazzy and Gazebo Harmonic, with a reproducible simulation baseline, visual perception, traversability estimation, motion control, and a browser-based dashboard.

1. Project at a Glance

This project targets autonomous navigation of a rugged unmanned ground vehicle in outdoor environments where the robot must understand the terrain in front of it, estimate where the vehicle can safely travel, command its drive system, and expose system state through an operator dashboard.

The implementation is being developed incrementally around a stable, testable simulation baseline. The current baseline deliberately separates what is already verified from subsystems that are still experimental or not yet implemented.

Current verified baseline

Subsystem

Current status

What is verified

Gazebo UGV simulation

🟢 Working

Main UGV world launches headlessly and spawns the sih_ugv model

ROS ↔ Gazebo bridge

🟢 Working

/camera, /camera_info, /imu, /cmd_vel are bridged

RGB camera

🟢 Working

640×480 RGB stream at approximately 30 Hz

IMU

🟢 Working

IMU stream at approximately 200 Hz

Differential drive

🟢 Working

ROS /cmd_vel produces physical rover motion in Gazebo

Odometry

🟢 Working

Gazebo odometry changes consistently with commanded motion

Perception baseline

🟢 Working

Camera-driven traversability mask is published

Traversability baseline

🟢 Working

Continuous score map and mean traversability are published

Dashboard

🟡 Functional / integration pending

Browser dashboard exists; motion-dependent integration still needs final verification

RGB-D localization

🟠 Experimental

Implementation and tests exist, but functional validation is not yet clean

Planner

🔴 Not implemented

Package scaffold only

Safety supervisor

🔴 Not implemented

Package scaffold only

Pan-tilt gimbal

🟡 Experimental

Separate experimental Gazebo model/world; not part of the stable baseline

The latest static audit also exposed and fixed a historical bringup regression: the original launch files had been deleted from the repository. They were restored from Git history and the package install rule was restored as well. The restored files are the historical headless/simulation launchers used by the baseline. fileciteturn161file0L280-L309

2. System Concept

The intended system is a rugged outdoor UGV with a sealed electronics body, four large wheels, camera-primary scene understanding, and eventual pan-tilt sensing.

The current stable simulation uses the original fixed front camera. The 2-axis pan-tilt camera exists only as an experimental model and is intentionally kept outside the production baseline until the core stack is stable.

High-level architecture

flowchart LR
    A[Gazebo Harmonic<br/>UGV + Outdoor World] --> B[ROS-Gazebo Bridge]
    B --> C[/camera]
    B --> D[/camera_info]
    B --> E[/imu]
    F[/cmd_vel] --> B
    B --> G[UGV Differential Drive]
    G --> H[UGV Motion]
    H --> I[Gazebo Odometry]

    C --> J[Perception Baseline]
    C --> K[Traversability Baseline]

    J --> L[/perception/traversability]
    K --> M[/traversability/score_map]
    K --> N[/traversability/mean_score]

    L --> O[Dashboard]
    M --> O
    N --> O
    E --> O
    I --> O

Data flow in one line

World → Camera / IMU → ROS Bridge → Perception / Traversability → Dashboard
                                  ↓
                               /cmd_vel
                                  ↓
                         Gazebo Differential Drive
                                  ↓
                              UGV Motion
                                  ↓
                              Odometry

3. Why a Camera-Primary Architecture?

Outdoor autonomous navigation has to deal with surfaces and conditions that are often poorly described by a simple geometric map: uneven terrain, vegetation, illumination changes, obstacles, and regions that are visually traversable but geometrically ambiguous.

This project therefore treats the forward camera as a primary environmental sensor for the baseline and derives a traversability representation directly from visual information.

The architecture is intentionally layered:

Simulation and sensing provide deterministic sensor streams.

Perception converts the image into a binary traversability representation.

Traversability estimation produces a continuous 0–1 score representation.

Motion control provides the vehicle actuation interface through /cmd_vel.

Dashboard provides operator-facing observability.

Localization, planning, and safety are treated as separate layers so that unfinished research components do not destabilize the verified baseline.

This separation makes every subsystem independently testable and makes failures easier to localize.

4. Hardware / Vehicle Concept

The target vehicle is a rugged 4-wheel differential-drive UGV concept with:

Four large outdoor wheels

Central sealed electronics enclosure

Front environmental sensing

Camera-primary navigation

IMU for inertial measurements

Differential-drive command interface

Future pan-tilt sensor head

The current Gazebo model contains the vehicle chassis, four wheels, differential-drive dynamics, IMU, and front RGB camera.

Conceptual visual

                         FRONT
                           ↑
                 ┌───────────────────┐
                 │   RGB CAMERA      │
                 │  future pan/tilt  │
                 └─────────┬─────────┘
                           │
              ┌────────────┴────────────┐
        ◉──────│   SEALED ELECTRONICS   │──────◉
        │      │       ENCLOSURE        │      │
        │      └────────────────────────┘      │
        │                                       │
        ◉───────────────────────────────────────◉

             4-wheel rugged outdoor UGV

Design intent: camera-first environmental understanding, robust mobility, clear subsystem boundaries, and eventual sensor-head actuation.

5. Software Stack

Layer

Technology

OS

Ubuntu 24.04.5 LTS

Environment

WSL2

Middleware

ROS 2 Jazzy

Simulator

Gazebo Harmonic / Gazebo Sim 8.15.0

GPU

NVIDIA GeForce RTX 5060 Laptop GPU

Rendering

Ogre2 / OpenGL through WSLg for graphical mode

Languages

Python + ROS 2 launch / CMake package infrastructure

Core perception

NumPy-based image processing

Localization experiment

OpenCV ORB + metric RGB-D processing

UI

Python web dashboard

The stable simulation is designed to work in headless mode, which avoids making GUI rendering a dependency for automated validation.

6. ROS 2 Packages

sih26126_ws/
└── src/
    ├── sih26126_msgs/
    ├── sih26126_bringup/
    ├── sih26126_sim/
    ├── sih26126_perception/
    ├── sih26126_localization/
    ├── sih26126_traversability/
    ├── sih26126_planner/
    ├── sih26126_safety/
    ├── sih26126_dashboard/
    └── sih26126_geometry/

Package responsibilities

Package

Responsibility

State

sih26126_msgs

Future/custom message definitions

Scaffold

sih26126_bringup

Reproducible launch files

Restored / working

sih26126_sim

Gazebo worlds and UGV model

Working

sih26126_perception

Visual traversability mask

Working baseline

sih26126_localization

RGB-D visual odometry experiment

Experimental

sih26126_traversability

Continuous traversability score

Working baseline

sih26126_planner

Navigation planning layer

Not implemented

sih26126_safety

Independent safety supervisor

Not implemented

sih26126_dashboard

Web-based system observability

Functional / integration pending

sih26126_geometry

3D ground/obstacle geometry baseline

Baseline / experimental

The static audit confirms that the planner and safety packages currently contain only package scaffolding and no ROS executables. fileciteturn161file0L163-L193 fileciteturn161file0L243-L258

7. Gazebo Simulation

Main world

src/sih26126_sim/worlds/ugv_test.sdf

Main UGV model

src/sih26126_sim/models/sih_ugv/sih_ugv.sdf

The UGV model validates independently as a valid SDF. The main world includes the model through Gazebo's resource search path. fileciteturn161file0L45-L59

Main sensing interfaces

Sensor / interface

ROS topic

Type

Baseline

Front RGB camera

/camera

sensor_msgs/msg/Image

640×480 RGB, ~30 Hz

Camera calibration

/camera_info

sensor_msgs/msg/CameraInfo

Available

IMU

/imu

sensor_msgs/msg/Imu

~200 Hz

Drive command

/cmd_vel

geometry_msgs/msg/Twist

Bidirectional bridge

Gazebo odometry

/model/sih_ugv/odometry (Gazebo)

Gazebo odometry

Available

The clean runtime baseline verified the camera stream, camera-info stream, IMU stream, command interface, and presence of the sih_ugv model. The camera measured roughly 29.3 Hz and the IMU roughly 200 Hz during validation. fileciteturn163file1L1-L1

8. ROS ↔ Gazebo Bridge

The restored historical bringup bridges the core sensor/control interfaces:

Gazebo                     ROS 2
────────────────────────────────────────────
/camera        ─────────→  /camera
/camera_info   ─────────→  /camera_info
/imu           ─────────→  /imu
/cmd_vel       ←────────→  /cmd_vel

The bridge is implemented through ros_gz_bridge and is deliberately kept small in the baseline.

Why this matters

A simulation is not useful for autonomy testing unless the sensor and command interfaces behave like a real robot's interfaces. The bridge therefore becomes the boundary between the simulated vehicle and the ROS autonomy stack.

9. Bringup and Reproducibility

The project previously lost its launch layer through commit 91d562f, which deleted both launch files. They were recovered exactly from Git history and reinstalled into the current workspace. fileciteturn161file0L298-L309

The restored headless launcher uses:

Gazebo server
      ↓
5-second startup delay
      ↓
ROS-Gazebo bridge

Headless launcher

src/sih26126_bringup/launch/headless_sim.launch.py

Graphical launcher

src/sih26126_bringup/launch/simulation.launch.py

The launch package also sets GZ_SIM_RESOURCE_PATH so the world can resolve its model assets from the simulation source tree.

Start the baseline

source /opt/ros/jazzy/setup.bash
source ~/sih26126_ws/install/setup.bash
cd ~/sih26126_ws

ros2 launch sih26126_bringup headless_sim.launch.py

Once running, the expected baseline graph is:

/sih26126_bridge

/camera
/camera_info
/imu
/cmd_vel

10. Perception Baseline

File:

src/sih26126_perception/sih26126_perception/perception_baseline.py

The current perception baseline is deliberately deterministic and interpretable.

Processing pipeline

flowchart TD
    A[RGB Camera Frame] --> B[RGB → Grayscale]
    B --> C[Ground ROI Selection]
    C --> D[Horizontal / Vertical Gradient]
    D --> E[Brightness + Smoothness Test]
    E --> F[Binary Traversability Mask]
    F --> G[/perception/traversability]

Current heuristic

The baseline focuses on the lower part of the image, where the terrain immediately in front of the rover is visible.

A pixel is treated as potentially traversable when:

brightness > threshold
AND
local gradient < threshold

The output is a mono8 image where traversable pixels are represented as 255 and non-traversable pixels as 0 within the relevant region.

Runtime behavior

The implementation has been observed processing the 640×480 camera stream at approximately 30 Hz on the working simulation baseline. fileciteturn163file0L1-L1

Why keep this baseline?

It provides a reference implementation against which future learned or geometric perception methods can be compared.

11. Traversability Estimation

File:

src/sih26126_traversability/sih26126_traversability/traversability_baseline.py

The traversability node produces a continuous 0–1 score, rather than only a binary decision.

Outputs

/traversability/score_map
/traversability/mean_score

Processing concept

flowchart LR
    A[RGB Image] --> B[Ground ROI]
    B --> C[Deterministic Score Function]
    C --> D[0..1 Score Map]
    D --> E[Mono8 Visualization]
    D --> F[Mean Score]

The score map is retained at the same image dimensions as the source frame, with the upper region marked as unknown and the ground ROI represented by an interpretable 0–255 visualization.

Synthetic validation

The deterministic implementation has been tested against synthetic bright, dark, and checkerboard inputs. The recorded synthetic outputs were:

bright_mean = 1.000000
dark_mean   = 0.400000
checker_mean= 0.300706
roi_start   = 192

Synthetic traversability tests: ALL OK

The live system has also produced mean scores around 0.998 in the nominal unobstructed baseline scene. fileciteturn163file7L1-L1

12. Motion Control and Odometry

The vehicle uses a differential-drive model and accepts:

/cmd_vel
geometry_msgs/msg/Twist

Verified control chain

sequenceDiagram
    participant ROS as ROS 2
    participant B as ros_gz_bridge
    participant G as Gazebo DiffDrive
    participant U as UGV
    participant O as Odometry

    ROS->>B: /cmd_vel
    B->>G: Gazebo Twist
    G->>U: Wheel velocities
    U-->>O: Vehicle motion

A controlled forward-command test demonstrated that a 0.5 m/s command applied for approximately 4 seconds produced roughly 2.06 m of simulated motion, consistent with the expected commanded displacement.

This closes one of the most important integration loops in the baseline:

ROS command → bridge → physics → vehicle motion → odometry

13. Dashboard

Package:

sih26126_dashboard

Executable:

sih26126_dashboard dashboard

The dashboard is a browser-based monitoring interface intended to expose the state of the autonomy stack to an operator.

Intended observability

┌──────────────────────────────────────────────────────┐
│                SIH26126 UGV DASHBOARD                │
├──────────────────────────────────────────────────────┤
│ Camera             ● LIVE                            │
│ Camera Info        ● LIVE                            │
│ IMU                ● LIVE                            │
│ Traversability     ● LIVE                            │
│ Command Interface  ● LIVE                            │
│                                                      │
│ Traversability: 0.998                                │
│                                                      │
│      [ Perception / Navigation Visualization ]       │
└──────────────────────────────────────────────────────┘

The dashboard was previously verified as a live browser interface on port 8080. The remaining integration task is to make its displayed state respond reliably to rover motion and the rest of the live graph.

The dashboard is currently treated as an observability layer, not as the autonomy decision-maker.

14. Localization — Experimental and Frozen for Now

File:

src/sih26126_localization/sih26126_localization/rgbd_odometry_baseline.py

The localization experiment uses RGB-D visual odometry concepts, including:

ORB feature detection

Feature matching

Metric 3D observations

PnP estimation

3D-to-3D rigid transform fallback

Pose accumulation

Experimental data path

flowchart LR
    A[RGB Camera] --> C[Feature Extraction]
    B[Metric RGB-D] --> C
    C --> D[Feature Matching]
    D --> E[PnP / 3D-3D]
    E --> F[Relative Motion]
    F --> G[Pose Accumulation]

However, this subsystem is not considered production-ready. Functional validation previously failed because the runtime code passed an unsupported seed keyword to ransac_rigid_transform(). fileciteturn162file1

Accordingly, localization remains outside the stable demo path until its implementation and tests are clean.

15. Geometry Baseline

Package:

sih26126_geometry

The geometry experiment extends the perception idea into a metric 3D representation of ground and obstacle structure.

This layer has previously been benchmarked for:

Ground estimation

Obstacle estimation

Negative / invalid geometry handling

Metric depth consistency

Confidence estimation

It is currently treated as a baseline/experimental capability rather than a mandatory dependency of the stable camera-only demonstration.

16. Planner and Safety — Deliberately Not Faked

Two packages exist in the workspace:

sih26126_planner
sih26126_safety

At the moment these are scaffolds, not finished autonomy layers. The static implementation audit found no planner/safety executable and only package-level files/tests. fileciteturn161file0L163-L193

This is intentional from a systems-engineering perspective: an incomplete planner or safety supervisor should not be presented as operational autonomy.

Intended future planner hierarchy

flowchart TD
    A[Perception] --> B[Traversability]
    B --> C[Local Cost Representation]
    C --> D[Local Planner]
    D --> E[Velocity Command]
    E --> F[/cmd_vel]

Intended safety hierarchy

flowchart TD
    A[Planner Command] --> C[Independent Safety Supervisor]
    B[Perception / Traversability] --> C
    D[Vehicle State / IMU / Odometry] --> C
    C -->|safe| E[/cmd_vel]
    C -->|unsafe| F[Stop / Recovery]

These are future architecture targets, not claims about the current implementation.

17. Experimental Pan-Tilt Camera

A separate experimental world/model has been created for a 2-axis camera head.

src/sih26126_sim/models/sih_ugv_gimbal/
src/sih26126_sim/worlds/gimbal_test.sdf

Conceptually:

                CAMERA
                  │
             ┌────┴────┐
             │ TILT AXIS│  ±90° target range
             └────┬────┘
                  │
             ┌────┴────┐
             │ PAN AXIS │  continuous rotation target
             └────┬────┘
                  │
             ┌────┴────┐
             │ CAMERA   │
             │  MAST    │
             └──────────┘

The gimbal SDF itself validates independently, but the gimbal world is experimental and untracked from the stable repository history at this stage. fileciteturn161file0L53-L59 fileciteturn161file0L295-L296

The gimbal will only enter the stable baseline after the core simulation and autonomy interfaces remain reliable with it.

18. Validation Strategy

The project uses a staged validation strategy rather than declaring the whole system “working” because one node launches.

Level 1 — Static validation

SDF parsing

Python syntax

ROS executable registration

package structure

installation correctness

Git state

Level 2 — Sensor validation

Camera messages arrive

Camera dimensions and encoding are correct

Camera info is available

IMU messages arrive

Rates remain within expected ranges

Level 3 — Algorithm validation

Perception output exists

Traversability output exists

Synthetic edge cases pass

Deterministic outputs remain interpretable

Level 4 — Control validation

/cmd_vel is connected

Commands reach Gazebo

UGV moves

Odometry changes

Level 5 — Application integration

Dashboard sees live topics

Dashboard changes with robot state

Perception and traversability remain alive during motion

One reproducible startup launches the intended stack

Level 6 — Autonomy validation

Future stages will add:

obstacle-aware planning

goal-directed navigation

independent safety supervision

recovery behavior

localization integration

gimbal control

broader outdoor scenario testing

19. Reproducible Baseline Checks

Check ROS packages

source /opt/ros/jazzy/setup.bash
source ~/sih26126_ws/install/setup.bash
ros2 pkg list | grep '^sih26126_' | sort

Check launch discovery

ros2 launch sih26126_bringup headless_sim.launch.py --show-args

Check ROS graph

ros2 node list
ros2 topic list

Inspect sensor streams

ros2 topic echo /camera --once
ros2 topic echo /camera_info --once
ros2 topic echo /imu --once

Inspect rates

ros2 topic hz /camera
ros2 topic hz /imu

Inspect command interface

ros2 topic info /cmd_vel

Inspect Gazebo model state

gz model --list

Send a controlled velocity command

ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"

Use Ctrl+C to stop publishing, then publish zero velocity:

ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"

20. Demonstration Flow

A clean demonstration should follow this sequence:

flowchart TD
    A[Launch UGV Simulation] --> B[Camera + IMU Start]
    B --> C[ROS Bridge Active]
    C --> D[Perception Active]
    D --> E[Traversability Active]
    E --> F[Dashboard Active]
    F --> G[Command UGV Forward]
    G --> H[Odometry Changes]
    H --> I[Dashboard Reflects Live State]

Recommended demo narrative

Start the stable simulation.

Show the UGV and sensor graph are alive.

Show the camera stream is publishing.

Show perception output.

Show traversability score/map.

Drive the vehicle using /cmd_vel.

Show odometry movement.

Show that the dashboard reflects the live state.

Clearly distinguish baseline capabilities from experimental/future autonomy modules.

21. Repository Structure

sih26126_ws/
│
├── src/
│   ├── sih26126_bringup/
│   │   ├── launch/
│   │   │   ├── headless_sim.launch.py
│   │   │   └── simulation.launch.py
│   │   └── CMakeLists.txt
│   │
│   ├── sih26126_sim/
│   │   ├── worlds/
│   │   │   ├── ugv_test.sdf
│   │   │   └── obstacle_test.sdf
│   │   └── models/
│   │       └── sih_ugv/
│   │           ├── model.sdf
│   │           └── sih_ugv.sdf
│   │
│   ├── sih26126_perception/
│   │   └── sih26126_perception/
│   │       ├── camera_monitor.py
│   │       └── perception_baseline.py
│   │
│   ├── sih26126_traversability/
│   │   └── sih26126_traversability/
│   │       └── traversability_baseline.py
│   │
│   ├── sih26126_localization/
│   │   └── sih26126_localization/
│   │       └── rgbd_odometry_baseline.py
│   │
│   ├── sih26126_geometry/
│   │   └── sih26126_geometry/
│   │       └── geometry_baseline.py
│   │
│   ├── sih26126_dashboard/
│   │   └── sih26126_dashboard/
│   │       └── dashboard_node.py
│   │
│   ├── sih26126_planner/
│   ├── sih26126_safety/
│   └── sih26126_msgs/
│
├── build/
├── install/
├── log/
└── README.md

22. Engineering Principles

1. Verified before integrated

A subsystem is not considered complete merely because its process starts. Its data path and outputs must be demonstrated.

2. Baseline before optimization

A deterministic baseline is retained before adding more advanced AI or autonomy methods.

3. Separate working and experimental code

Localization and gimbal work are not allowed to silently destabilize the working simulation.

4. Reproducibility matters

The project keeps launch files, world definitions, bridge interfaces, and validation commands explicit so that another developer can reproduce the environment.

5. Do not hide unfinished autonomy

Planner and safety are currently scaffolds. They are presented as such instead of being described as completed functionality.

23. Current Known Limitations

GUI rendering under WSLg

Graphical Gazebo can be unreliable under WSLg even when the headless simulation is healthy. Therefore the core validation path is intentionally headless.

Localization

RGB-D visual odometry remains experimental and requires additional functional validation before it belongs in the main navigation loop.

Planner

A complete global/local planning pipeline is not yet integrated.

Safety

An independent command supervisor with stop/recovery behavior is not yet integrated.

Dashboard integration

The dashboard application exists and can show live topics, but motion-dependent end-to-end integration still needs to be finalized.

Gimbal

The pan-tilt camera remains experimental and is not part of the stable demo baseline.

24. Roadmap

gantt
    title SIH26126 Development Roadmap
    dateFormat  YYYY-MM-DD
    axisFormat  %b %d

    section Baseline
    Gazebo + bridge                 :done, a1, 2026-09-01, 7d
    Perception baseline             :done, a2, 2026-09-05, 7d
    Traversability baseline        :done, a3, 2026-09-08, 7d
    Motion + odometry verification :done, a4, 2026-09-15, 5d
    Bringup restoration            :done, a5, 2026-09-22, 3d

    section Integration
    Dashboard motion integration   :active, b1, 2026-09-25, 7d
    One-shot full-stack bringup    :b2, 2026-09-29, 7d

    section Autonomy
    Localization stabilization    :c1, 2026-10-06, 14d
    Local planner                 :c2, 2026-10-13, 14d
    Safety supervisor             :c3, 2026-10-20, 14d
    Pan-tilt integration          :c4, 2026-10-27, 14d

Dates above are a planning illustration, not a committed schedule.

25. Future Target Architecture

The eventual integrated autonomy stack is intended to look like:

flowchart TB
    S[Sensors<br/>RGB Camera + IMU + Future Depth] --> P[Perception]
    P --> T[Traversability]
    T --> C[Cost / Local Terrain Representation]
    S --> L[Localization]
    L --> C
    C --> PL[Local Planner]
    PL --> SP[Safety Supervisor]
    SP --> CMD[/cmd_vel]
    CMD --> U[UGV]
    U --> S

    T --> D[Dashboard]
    L --> D
    SP --> D
    U --> D

This architecture is intended to provide closed-loop autonomy while maintaining independent observability and safety boundaries.

26. Validation Snapshot

The stable baseline has demonstrated:

UGV model                PASS
ROS-Gazebo bridge        PASS
Camera                   PASS
Camera info              PASS
IMU                      PASS
cmd_vel interface        PASS
UGV motion               PASS
Odometry response        PASS
Perception baseline      PASS
Traversability baseline  PASS

Localization             EXPERIMENTAL
Planner                  NOT IMPLEMENTED
Safety                   NOT IMPLEMENTED
Gimbal                   EXPERIMENTAL
Dashboard motion loop    PENDING FINAL INTEGRATION

The static audit also confirmed the ROS executables currently registered for the implemented baseline nodes: perception, traversability, localization experiment, and dashboard. Planner and safety do not currently register executables. fileciteturn161file0L243-L259

27. Contributors and Acknowledgment

Developed for Smart India Hackathon 2026 — SIH26126.

Sponsor: Bharat Electronics Limited (BEL)

Domain: Vision-based autonomous navigation for unmanned ground vehicles in outdoor environments.

28. License

Add the repository's final project license here once the team has selected and committed the intended license.

Final Note

This repository is intentionally transparent about project maturity. The core simulated sensing, ROS bridge, perception, traversability, differential-drive control, and odometry path are the current engineering baseline. Localization, planning, safety, and gimbal integration are kept separate until they meet the same validation standard.

That makes the project easier to debug, easier to reproduce, and easier to extend toward the final autonomous UGV architecture.
