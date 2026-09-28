# SIH26126 — MASTER CODEX ENGINEERING SPECIFICATION

You are the primary autonomous software engineer responsible for completing and validating the entire SIH26126 project in this repository.

Repository:
~/sih26126_ws

Project:
SIH26126 — Vision Based Autonomous Navigation for Unmanned Ground Vehicle for Outdoor Environment

Sponsor:
Bharat Electronics Limited (BEL)

Category:
Software

Theme:
Smart Automation

Primary platform:
Ubuntu 24.04 / WSL2
ROS 2 Jazzy
Gazebo Harmonic-class
Python 3.12
colcon
pytest
ros_gz_bridge

Git branch:
feat/localization-repair

Remote:
https://github.com/bhargavshantharaju/sih26126_ws.git


============================================================
1. PRIMARY OBJECTIVE
============================================================

Finish this repository as a coherent, reproducible, technically defensible autonomous-navigation prototype.

Do not merely make it compile.

The finished system must demonstrate a complete closed-loop architecture:

CAMERA / DEPTH / IMU
        ->
PERCEPTION
        ->
3D GEOMETRY
        ->
TERRAIN / TRAVERSABILITY
        ->
VISUAL LOCALIZATION / ODOMETRY
        ->
GLOBAL GOAL PLANNING
        ->
LOCAL OBSTACLE AVOIDANCE
        ->
SAFETY SUPERVISOR
        ->
/cmd_vel
        ->
GAZEBO UGV
        ->
updated sensor observations
        ->
closed loop

The official success objective is collision-free Point-A-to-Point-B navigation in an outdoor/GPS-denied scenario.

You must prioritize functional correctness and reproducibility over adding superficial features.


============================================================
2. NON-NEGOTIABLE ENGINEERING RULES
============================================================

1. Work directly in the existing repository.
2. Inspect the existing source before changing it.
3. Do not throw away working functionality without evidence.
4. Do not rewrite the entire repository from scratch.
5. Preserve the package names and existing public ROS interfaces unless there is a concrete reason to improve them.
6. Do not silently fake functionality.
7. Do not replace a difficult subsystem with a stub that merely makes tests pass.
8. Do not claim a capability is working unless you actually validate it.
9. When a test fails, diagnose the failure and fix the cause before moving on.
10. Use small, testable changes.
11. After meaningful changes, rebuild affected packages.
12. Before finalizing, perform a clean full-workspace build and validation.
13. Remove stale/generated junk from source control where appropriate.
14. Do not commit build/, install/, log/, __pycache__, .pytest_cache, *.pyc, temporary files, ROS logs, or machine-specific artifacts unless the repository already intentionally tracks them.
15. Do not hardcode /home/venom paths when the project can use package/share paths, relative paths, or ROS package discovery.
16. Avoid broad destructive process-killing commands during validation.
17. Do not use fake test fixtures to hide integration failures.
18. Do not change the problem into a simpler problem.
19. Do not remove obstacle avoidance merely because it is currently failing.
20. Do not remove the safety supervisor or allow planners to bypass it.
21. Do not make the dashboard mandatory for autonomous control.
22. The system must remain usable headlessly.
23. Treat runtime evidence as more authoritative than assumptions.
24. Distinguish unit-test correctness from integration correctness.


============================================================
3. CURRENT REPOSITORY KNOWLEDGE
============================================================

The repository currently contains these packages:

sih26126_msgs
sih26126_dashboard
sih26126_geometry
sih26126_localization
sih26126_perception
sih26126_planner
sih26126_safety
sih26126_sensor_contract
sih26126_sim
sih26126_traversability
sih26126_bringup

Important existing components include:

- SensorHealth custom message
- sensor-health / freshness / validity / synchronization logic
- geometry_baseline
- geometry_terrain / terrain_geometry
- terrain_classifier
- perception_baseline
- traversability_baseline
- terrain_aware_traversability
- rgbd_odometry_baseline
- planner_baseline
- goal_path_planner
- local_path_selector
- safety_supervisor
- simulation worlds
- full_stack.launch.py
- simulation.launch.py
- headless_sim.launch.py
- dashboard


============================================================
4. KNOWN EXISTING STRENGTHS
============================================================

These capabilities already exist to some degree and should be preserved:

A. Full ROS 2 package structure.
B. Gazebo integration.
C. ROS/Gazebo bridge.
D. Camera and depth streams.
E. Metric point-cloud processing.
F. Terrain geometry logic.
G. Terrain classification.
H. Traversability scoring.
I. Goal-oriented planning.
J. Local candidate-path planning.
K. Safety supervisor.
L. Sensor health contract.
M. Dedicated simulation worlds.
N. Unit tests for multiple subsystems.
O. Safety owns final /cmd_vel.

One particularly important architecture rule already demonstrated:

/cmd_vel must have exactly one authoritative publisher:
safety_supervisor

The planner may publish recommended commands, but the safety supervisor remains the final authority.


============================================================
5. KNOWN PROBLEMS THAT MUST BE SOLVED
============================================================

The following issues have already been observed and must be explicitly investigated and fixed.

------------------------------------------------------------
5.1 TERRAIN GEOMETRY RUNTIME ERROR
------------------------------------------------------------

Observed runtime error:

Terrain geometry processing failed:
RcutilsLogger.info() takes 2 positional arguments but 7 were given

This indicates incorrect ROS logger API usage.

Fix the actual logging call.

Then verify that terrain_geometry runs continuously without this exception and publishes valid outputs.

Do not simply suppress the exception.


------------------------------------------------------------
5.2 INTEGRATED OBSTACLE AVOIDANCE IS NOT YET VERIFIED
------------------------------------------------------------

The latest obstacle-world test showed the robot moving toward/through the benchmark obstacle while the local planner repeatedly reported approximately:

local angle=0.0 deg
obstacle=3.00 m
obstacle_score=1.000

The robot subsequently stopped when navigation became inactive.

This means end-to-end obstacle avoidance has NOT been proven.

However, lower-level local-path-selector tests already demonstrated that obstacle-clearance logic can detect representative obstacle geometry.

Therefore, investigate the complete chain:

/depth_camera/points
    ->
PointCloud2 parsing
    ->
coordinate/frame interpretation
    ->
filtering
    ->
corridor extraction
    ->
obstacle clearance
    ->
candidate scores
    ->
selected candidate
    ->
recommended command
    ->
safety supervisor
    ->
/cmd_vel
    ->
robot motion

Instrument intermediate values where necessary.

Do not blindly tune constants until the data path is understood.


------------------------------------------------------------
5.3 SOURCE / BUILD / INSTALL DRIFT
------------------------------------------------------------

Previous repository states showed inconsistencies between source setup.py files and installed executables.

Examples included missing or inconsistent console_scripts for:

terrain_geometry
terrain_classifier
terrain_aware_traversability
planner_baseline
goal_path_planner
local_path_selector
rgbd_odometry_baseline
safety_supervisor

Audit every ament_python package.

For every executable:
- ensure console_scripts entry exists
- ensure package is importable
- ensure executable is installed
- ensure `ros2 pkg executables PACKAGE` finds it
- ensure launch files reference the correct executable

Rebuild from source after fixes.

Do not rely on stale install/ artifacts.


------------------------------------------------------------
5.4 PACKAGE.XML DEPENDENCIES
------------------------------------------------------------

Audit all package.xml files.

Dependencies used at runtime must be declared correctly.

Do not leave TODO package descriptions, fake maintainer data, or missing runtime dependencies if they can be fixed properly.

At minimum audit:
- rclpy
- std_msgs
- geometry_msgs
- nav_msgs
- sensor_msgs
- sensor_msgs_py
- tf2 / tf2_ros where used
- ros_gz_bridge-related runtime requirements
- custom message dependencies
- pytest/test dependencies


------------------------------------------------------------
5.5 TEST DISCOVERY
------------------------------------------------------------

Previous direct pytest runs passed in several packages, while colcon test reported:

NO TESTS RAN

for packages including:
- dashboard
- geometry
- perception
- traversability

This is a packaging/test-discovery defect.

Fix test discovery so that:

colcon test

actually executes the package tests.

Do not solve this by deleting tests.

Ensure direct pytest and colcon test agree.


------------------------------------------------------------
5.6 DASHBOARD PORT CONFLICT
------------------------------------------------------------

Previous integrated runs showed:

OSError: [Errno 98] Address already in use

on port 8080.

Make dashboard startup robust.

Preferred design:
- configurable port parameter
- documented default
- clean handling when the port is unavailable
- do not make dashboard failure terminate autonomous navigation

Headless navigation must work without dashboard.


------------------------------------------------------------
5.7 LAUNCH FILE WORLD SELECTION
------------------------------------------------------------

Audit:

src/sih26126_bringup/launch/full_stack.launch.py

The stack must support selecting a simulation world through launch arguments.

Required style:

ros2 launch sih26126_bringup full_stack.launch.py \
    world:=/path/to/obstacle_test.sdf

Verify that the requested world is actually launched.

Do not hardcode one world as the only behavior.

Also audit simulation.launch.py and headless_sim.launch.py for consistency.


------------------------------------------------------------
5.8 HARDCODED WORKSPACE PATHS
------------------------------------------------------------

Replace fragile absolute paths such as:

/home/venom/sih26126_ws/...

where package-share APIs or launch substitutions can be used.

The repository must remain portable to another machine/user where practical.

Machine-specific absolute paths are acceptable only where genuinely required by the environment and documented.


============================================================
6. OBSTACLE DETECTION REQUIREMENTS
============================================================

This is the highest priority functional milestone.

The obstacle benchmark must produce a measurable sequence.

Expected behavior:

1. Obstacle far away:
   large/infinite clearance

2. Obstacle enters sensing corridor:
   finite clearance

3. Straight-ahead candidate becomes unsafe:
   candidate score decreases

4. Side candidate remains feasible:
   side candidate wins

5. Local steering changes:
   non-zero angular command

6. Vehicle diverts before collision

7. Vehicle passes obstacle

8. Obstacle clears from corridor

9. Vehicle converges back toward goal

10. Vehicle reaches Point B

The exact numerical thresholds may be changed only after measurement.


============================================================
7. DO NOT MISINTERPRET DISPLAYED CLEARANCE
============================================================

There has historically been a distinction between raw clearance and display/clamped clearance.

Do not assume:

obstacle=3.00 m

means a physical obstacle was detected at 3 m.

Inspect the actual raw return and candidate score.

If clearance is infinity and the UI/log clamps it to 3.00 m, make the diagnostic logging clearer.

The final logs should distinguish:

raw_clearance
effective_clearance
obstacle_score
selected_candidate
candidate_scores


============================================================
8. LOCAL PLANNER REQUIREMENTS
============================================================

Maintain a lightweight candidate-based planner.

Representative candidate directions may include:

-25.0 deg
-16.7 deg
-8.3 deg
0.0 deg
8.3 deg
16.7 deg
25.0 deg

Evaluate candidates using a reasonable combination of:

- goal alignment
- terrain traversability
- obstacle clearance
- steering smoothness
- speed suitability
- current heading
- confidence

Do not choose a candidate using goal angle alone.

Keep the implementation understandable and deterministic.


============================================================
9. SAFETY SUPERVISOR REQUIREMENTS
============================================================

The safety supervisor is authoritative.

It must be the only final /cmd_vel publisher.

Required behaviors include:

SAFE_COMMAND
OBSTACLE_STOP
STALE_NAVIGATION_STATE
WAITING_FOR_NAVIGATION_STATE
NAVIGATION_NOT_ACTIVE
invalid command handling
sensor/data freshness handling where applicable
command clamping where appropriate

Fail-safe rule:

When critical information is missing or stale, the robot stops.

Do not make safety dependent solely on a neural-network confidence value.


============================================================
10. SENSOR CONTRACT REQUIREMENTS
============================================================

Audit the sensor contract subsystem.

For camera, depth, odometry, and IMU where used, define/check:

- message arrival
- freshness
- validity
- timestamps
- synchronization where necessary
- frame IDs
- NaN/Inf handling
- empty-cloud behavior
- unexpected field layouts

Do not allow a stale frame to appear equivalent to a valid current frame.


============================================================
11. POINTCLOUD REQUIREMENTS
============================================================

The current simulated depth cloud is PointCloud2 with XYZ float32 fields.

The implementation must correctly handle:

x = forward
y = lateral
z = vertical

for the actual camera frame used by the benchmark.

Do not assume ROS optical-frame conventions without checking the actual frame and transform.

Explicitly verify:

- width
- height
- point_step
- field offsets
- datatype
- frame_id
- finite values
- transform to the planning frame

If a transform is necessary, implement it correctly rather than embedding guessed axis swaps.


============================================================
12. TERRAIN GEOMETRY REQUIREMENTS
============================================================

Terrain geometry should estimate useful properties such as:

- ground plane
- slope
- roughness
- local height variation
- point validity
- confidence
- neighbor consistency

It must robustly handle:
- NaN
- Inf
- sparse neighborhoods
- missing depth
- invalid points
- edge regions

Do not use invalid neighboring pixels when estimating gradients.


============================================================
13. TERRAIN CLASSIFICATION
============================================================

Maintain a deterministic baseline classifier if ML classification is not yet available.

The classifier should produce interpretable outputs.

Possible states include:

FLAT
ROUGH
SLOPE
OBSTACLE
UNKNOWN

Where appropriate, publish:

- class
- class confidence
- slope
- roughness
- geometry confidence
- valid ratio

Do not fabricate semantic classes from unsupported information.


============================================================
14. TRAVERSABILITY
============================================================

The terrain-aware traversability layer should combine:

semantic information where available
+
geometry
+
slope
+
roughness
+
obstacle state
+
confidence

into a meaningful score.

A traversability score must not remain near 1.0 merely because no semantic model is installed.

Unknown terrain should not automatically become fully traversable.

Make the behavior explicit and conservative.


============================================================
15. NEGATIVE OBSTACLE SUPPORT
============================================================

This project explicitly mentions ditches as hazards.

Implement a dedicated negative-obstacle strategy where feasible.

Examples:
- ditch
- drop-off
- trench
- hole
- sudden depth discontinuity

Useful geometric cues:
- unexpected ground-plane discontinuity
- missing expected ground points
- large downward depth transition
- local elevation discontinuity
- semantic water/ditch evidence if available

Do not claim this is solved unless an actual benchmark test demonstrates it.

Create a dedicated simulation benchmark if needed.


============================================================
16. LOCALIZATION
============================================================

The current RGB-D odometry baseline is valuable and must remain functional.

Improve robustness where practical.

Requirements:
- valid pose publishing
- sensible frame IDs
- timestamp consistency
- no NaN pose
- clear pose confidence/health
- explicit stale/lost state

Where possible, prepare the architecture for visual-inertial SLAM without making the project dependent on an unverified external stack.

Do not install large external systems merely for appearance.


============================================================
17. GLOBAL GOAL PLANNING
============================================================

The goal planner must:

- consume pose
- consume traversability/terrain information
- publish a meaningful path/reference
- compute distance to goal
- publish navigation state
- publish recommended speed
- publish recommended command

It must NOT directly own /cmd_vel.

Navigation states must be well-defined and documented.

GOAL_REACHED should be based on an explicit goal-tolerance condition.


============================================================
18. CONTROL AND SPEED
============================================================

Speed must be reduced when:

- terrain is poor
- confidence decreases
- obstacle clearance decreases
- turning becomes aggressive
- localization confidence decreases

Speed must increase only when the system has sufficient confidence.

Ensure angular and linear commands stay within safe physical limits.


============================================================
19. GAZEBO / ROS BRIDGE
============================================================

Audit bridge mappings carefully.

Expected topics include at least:

/imu
/camera
/camera_info
/depth_camera/points
/cmd_vel
/model/sih_ugv/odometry

Use correct ROS/Gazebo message mappings.

Verify:
- one expected publisher per sensor
- no duplicate bridge instances
- correct command direction
- correct odometry direction


============================================================
20. NODE GRAPH REQUIREMENT
============================================================

A clean full-stack run should produce one logical instance of:

/sih26126_bridge
/perception_baseline
/traversability_baseline
/terrain_geometry
/terrain_classifier
/terrain_aware_traversability
/goal_path_planner
/local_path_selector
/safety_supervisor
/sensor_contract

Dashboard may run separately and must not be required for navigation.

Do not allow duplicate nodes from stale sessions to contaminate validation.


============================================================
21. DUPLICATE PROCESS PROTECTION
============================================================

Do not build brittle process management into the product.

During testing, use controlled process lifecycle.

Do not use indiscriminate:

pkill -f ...

as the normal validation strategy.

Prefer:
- launch lifecycle
- recorded PIDs
- process groups where appropriate
- clean shutdown
- ROS launch actions
- explicit stale-process detection

Document test lifecycle.


============================================================
22. TEST SUITE REQUIREMENTS
============================================================

Create/fix tests for:

A. geometry math
B. terrain geometry
C. terrain classification
D. traversability
E. goal planning
F. local path selection
G. obstacle clearance
H. safety supervisor
I. sensor contract
J. localization sanity
K. launch configuration
L. package/executable discovery where practical

Important local planner regression cases:

1. no obstacle
2. far obstacle
3. close obstacle
4. obstacle centered
5. obstacle left
6. obstacle right
7. invalid points
8. empty point cloud
9. NaN/Inf
10. transformed point cloud
11. all candidates blocked
12. side route available


============================================================
23. FULL-COLCON VALIDATION
============================================================

Eventually the following must be run:

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

then:

source install/setup.bash

then:

colcon test --event-handlers console_direct+

then:

colcon test-result --verbose

No important package should report:

NO TESTS RAN

unless it genuinely has no tests and that is documented.

Build must exit successfully.

Core tests must pass.


============================================================
24. DIRECT PYTEST VALIDATION
============================================================

Direct pytest should also be run for individual packages when debugging.

Existing evidence includes:

18 local-path-selector tests passing.

Preserve and expand that regression coverage.


============================================================
25. RUNTIME VALIDATION
============================================================

The final runtime validation must include:

A. clean environment
B. build
C. launch
D. node graph check
E. topic publisher/subscriber check
F. sensor rates/data
G. terrain outputs
H. planner outputs
I. local planner outputs
J. safety outputs
K. /cmd_vel ownership
L. obstacle approach
M. obstacle avoidance
N. obstacle pass
O. goal completion
P. clean shutdown


============================================================
26. DEFINITIVE OBSTACLE TEST
============================================================

Use:

src/sih26126_sim/worlds/obstacle_test.sdf

unless a superior benchmark is created.

Do not accept a test that merely checks that the system launches.

The test must provide evidence that the obstacle caused a trajectory change.

Collect:

timestamp
x
y
yaw
goal distance
local steering angle
raw clearance
effective clearance
obstacle score
selected candidate
linear command
angular command
terrain score
safety state
safety reason

A successful result requires:

obstacle appears
AND
straight path becomes penalized
AND
alternative trajectory selected
AND
robot changes lateral trajectory
AND
robot avoids contact
AND
robot eventually reaches goal


============================================================
27. NEGATIVE OBSTACLE TEST
============================================================

Create or extend a simulation world containing a clear negative obstacle.

At minimum test:
- shallow drop
- deeper ditch

Expected:
- system detects dangerous geometry
- terrain/traversability cost increases
- local planner avoids or stops
- safety supervisor stops if no safe candidate exists


============================================================
28. DYNAMIC OBSTACLE TEST
============================================================

Where feasible, add at least one dynamic obstacle scenario.

Expected:
- obstacle enters path after navigation starts
- obstacle is detected
- local path changes
- robot does not collide
- navigation resumes toward the goal


============================================================
29. FAILURE INJECTION
============================================================

Test safety behavior by deliberately simulating:

- frozen camera
- stale depth
- stale odometry
- invalid point cloud
- missing navigation state
- invalid command
- planner failure

Expected safety response:
STOP or other explicitly safe behavior.

Never allow stale data to continue controlling the vehicle indefinitely.


============================================================
30. PERFORMANCE MEASUREMENT
============================================================

Instrument and report actual values for:

- perception latency
- geometry latency
- traversability latency
- local planning frequency
- command update rate
- sensor frequency
- emergency-stop response
- end-to-end control latency

Do not invent target numbers as measured results.

Targets may be documented separately from measurements.


============================================================
31. CODE QUALITY
============================================================

Run:

flake8
pep257

or package-equivalent ROS ament checks.

Fix actual issues rather than suppressing everything.

Keep:
- imports clean
- docstrings meaningful
- functions reasonably sized
- constants named
- logging useful
- types sensible
- error paths explicit

Do not spend disproportionate time beautifying code while a functional failure remains unresolved.


============================================================
32. LICENSING
============================================================

Audit dependencies and models.

Do not claim that all candidate models are Apache/BSD merely because a project is open source.

For every newly introduced external dependency:
- identify license
- identify whether pretrained weights have separate license terms
- document the dependency
- avoid non-commercial restrictions when commercial productization is a stated goal unless clearly documented as research-only

Prefer minimal, well-understood dependencies.


============================================================
33. DOCUMENTATION REQUIREMENTS
============================================================

Create/update:

README.md

docs/ARCHITECTURE.md
docs/TESTING.md
docs/VALIDATION.md
docs/SAFETY.md
docs/TROUBLESHOOTING.md
docs/REPRODUCIBILITY.md
docs/LICENSE_AUDIT.md

Documentation must reflect what is actually implemented.

Include:
- architecture diagram in text/mermaid where useful
- node responsibilities
- topic graph
- launch commands
- world selection
- test commands
- expected outputs
- known limitations
- measured results
- unresolved issues if any


============================================================
34. READ THE EXISTING DOCS BEFORE MODIFYING
============================================================

Inspect all existing files in:

docs/
README.md

Do not overwrite valuable existing documentation without reading it first.

Preserve useful historical context where relevant.


============================================================
35. CLEAN REPOSITORY REQUIREMENTS
============================================================

Inspect:

git status
git diff
git ls-files

Before finalization:
- remove empty accidental files
- remove obvious temporary files
- remove generated caches
- ensure .gitignore is appropriate
- ensure important source/test/docs are tracked
- avoid committing install/build/log artifacts


============================================================
36. GIT REQUIREMENTS
============================================================

Do not lose existing user work.

Before major modifications:
- inspect git status
- inspect diff where needed

At completion:
- show git diff --stat
- show git status
- create a focused commit only if all required validation passes

Do not push blindly.

Before any push:
- verify branch
- verify remote
- verify commit
- verify working tree

If push is appropriate, push only:

origin feat/localization-repair

Do not force-push.
Do not rewrite history.
Do not delete remote branches.


============================================================
37. DEVELOPMENT PRIORITY ORDER
============================================================

Priority 1:
Make source/build/install consistent.

Priority 2:
Fix terrain_geometry runtime exception.

Priority 3:
Fix test discovery and packaging.

Priority 4:
Trace and fix integrated obstacle detection.

Priority 5:
Prove obstacle avoidance in the obstacle world.

Priority 6:
Make goal completion reproducible.

Priority 7:
Improve safety fault injection.

Priority 8:
Improve localization robustness.

Priority 9:
Implement/strengthen negative-obstacle support.

Priority 10:
Improve dashboard reliability.

Priority 11:
Add real-data/perception improvements that are actually feasible.

Priority 12:
Documentation and final cleanup.

Do NOT reverse this order merely because adding an ML model looks more impressive.


============================================================
38. DO NOT OVERBUILD
============================================================

You are not being asked to create a giant research framework.

Do not:
- introduce unnecessary microservices
- add Kubernetes
- add cloud services
- add unrelated AI agents
- install massive ML dependencies without need
- introduce complex databases
- introduce unnecessary GUI systems
- replace ROS 2 with another middleware

Keep the system lightweight, explainable and deployable.


============================================================
39. DEFINITION OF DONE
============================================================

The project is DONE only when the following are all true:

[ ] All 11 expected ROS packages build successfully.

[ ] Required executables exist and are discoverable.

[ ] Package.xml runtime dependencies are correct.

[ ] setup.py console_scripts are correct.

[ ] Full source rebuild works from a fresh shell.

[ ] colcon test executes package tests correctly.

[ ] No important package reports accidental NO TESTS RAN.

[ ] Direct pytest suites pass.

[ ] flake8/pep257 issues are reduced to intentional/documented exceptions only.

[ ] terrain_geometry runs without the RcutilsLogger exception.

[ ] depth point cloud is correctly interpreted in the planning frame.

[ ] local obstacle clearance produces finite values for the benchmark obstacle.

[ ] integrated obstacle scoring actually penalizes blocked candidates.

[ ] local planner changes steering when the obstacle blocks the current path.

[ ] safety supervisor remains the sole /cmd_vel publisher.

[ ] robot diverts around the obstacle.

[ ] robot passes the obstacle without contact.

[ ] robot continues toward the goal.

[ ] robot reaches Point B.

[ ] safety stops the robot for stale/lost navigation conditions.

[ ] negative-obstacle behavior is either implemented and tested or explicitly documented as an incomplete research limitation; do not claim it is solved if it is not.

[ ] dashboard failure does not prevent headless autonomy.

[ ] requested Gazebo world can be selected through launch arguments.

[ ] duplicate-node contamination is eliminated from official validation.

[ ] documentation matches implementation.

[ ] repository does not rely on stale install/build state.

[ ] git status is understood.

[ ] final validation results are recorded in docs/VALIDATION.md.


============================================================
40. REQUIRED FINAL VALIDATION COMMANDS
============================================================

Use a clean shell state where practical:

source /opt/ros/jazzy/setup.bash

Then:

colcon build --symlink-install

Then:

source install/setup.bash

Then:

ros2 pkg list | grep '^sih26126_' | sort

Then enumerate executables:

for p in $(ros2 pkg list | grep '^sih26126_' | sort); do
    echo "=== $p ==="
    ros2 pkg executables "$p" || true
done

Then:

colcon test --event-handlers console_direct+

Then:

colcon test-result --verbose


============================================================
41. REQUIRED RUNTIME CHECKS
============================================================

Perform a controlled obstacle-world run.

Use the actual full stack.

Check:

ros2 node list

ros2 topic info /cmd_vel -v

ros2 topic info /depth_camera/points -v

ros2 topic info /model/sih_ugv/odometry -v

ros2 topic echo /planner/navigation_state --once

ros2 topic echo /planner/local_obstacle_score --once

ros2 topic echo /planner/local_obstacle_clearance --once

ros2 topic echo /planner/local_steering_angle --once

ros2 topic echo /safety/state --once

ros2 topic echo /safety/reason --once

Verify /cmd_vel has one publisher:
safety_supervisor


============================================================
42. LOGGING / OBSERVABILITY
============================================================

Make runtime logs concise but sufficient.

For local planner periodic diagnostic logging include:

angle
raw clearance
effective clearance
obstacle score
terrain score
goal alignment
selected candidate
speed

For safety logging include:

state
reason
command

For terrain logging include:

class
class confidence
slope
roughness
geometry confidence
valid ratio


============================================================
43. IF AN EXTERNAL DEPENDENCY IS MISSING
============================================================

Do not immediately redesign the project.

First:
1. verify whether the dependency is actually required
2. check whether an existing local implementation can satisfy the need
3. install only if appropriate
4. document it
5. test it

If internet access is unavailable, continue with offline fixes where possible and clearly document what could not be verified.


============================================================
44. IF A TEST FAILS
============================================================

Use this loop:

FAIL
 ->
read failure
 ->
locate code
 ->
reproduce narrowly
 ->
fix root cause
 ->
run focused test
 ->
run affected package tests
 ->
run full validation

Do not simply weaken the assertion to get green tests.


============================================================
45. IF THE FULL END-TO-END DEMO STILL FAILS
============================================================

Do not declare the project complete.

Instead:
- identify exact failed stage
- add instrumentation
- isolate the subsystem
- fix it
- rerun

The final response must distinguish:
PASS
PARTIAL
UNVERIFIED
BLOCKED

Do not convert UNVERIFIED into PASS.


============================================================
46. FINAL DELIVERABLES
============================================================

At completion, the repository should contain:

working ROS 2 source
working launch files
working simulation worlds
working tests
working safety system
working local obstacle avoidance
working documentation
validation evidence
clean package metadata
reproducible build

Create:

docs/VALIDATION.md

with:
- environment
- build result
- package result
- executable result
- test result
- runtime graph
- obstacle benchmark
- goal benchmark
- safety fault tests
- measured timings
- known limitations


============================================================
47. FINAL REPORT TO USER
============================================================

At the very end, report:

1. What was fixed.
2. What was added.
3. What tests passed.
4. Exact build result.
5. Exact colcon test result.
6. Exact obstacle benchmark result.
7. Exact goal completion result.
8. Exact safety result.
9. Any remaining limitations.
10. Git status.
11. Commit hash if committed.
12. Whether anything was pushed.

Do not claim success unless the acceptance criteria above have actually been checked.


============================================================
48. OPERATING MODE
============================================================

Work autonomously.

Do not repeatedly ask for permission to:
- inspect files
- edit source
- run tests
- run builds
- inspect ROS topics
- inspect Git history
- create documentation

You have authority to modify the repository to complete the task.

However:
- do not destroy user work
- do not force-push
- do not erase Git history
- do not hide failures
- do not fabricate results

When several approaches are possible, choose the simplest approach that preserves the architecture and provides measurable evidence.

Start immediately.


============================================================
49. FIRST ACTIONS
============================================================

Before editing anything, execute and inspect:

pwd
git status --short
git branch --show-current
git log --oneline -10
git remote -v

find src -maxdepth 2 -type f | sort

Then inspect:
README.md
docs/
all package.xml
all setup.py
all launch files
planner files
safety files
geometry files
traversability files
localization files
simulation worlds

Then create a written internal implementation plan.

Then execute the priorities above one by one.

Do not stop at the planning stage.


============================================================
50. FINAL ABSOLUTE REQUIREMENT
============================================================

Do not optimize for saying "done".

Optimize for producing a repository that another engineer can clone, build, launch and independently verify.

The final acceptance test is not:

"the code looks good."

The final acceptance test is:

"A cleanly built system can autonomously navigate from Point A to Point B in the benchmark environment, react to an obstacle by selecting a safe alternative trajectory, pass the obstacle without collision, and stop safely when critical navigation information is invalid or stale."

That is the project you must finish.
