# AI Development Rules — SIH26126

## Platform

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python + C++
- OpenCV
- PyTorch
- Git

## Architecture

- Camera-primary perception
- GPS is NOT required for core navigation
- LiDAR must remain optional
- Simulation before hardware
- Baseline before optimization

## Engineering rules

1. Do not invent ROS topics, services, actions or APIs.
2. Verify package/API availability before using it.
3. Keep ROS package responsibilities separate.
4. Do not modify Nav2 internals unless necessary.
5. Prefer plugins, nodes and adapters around existing systems.
6. Every major algorithm needs a measurable baseline.
7. Do not hide failures with arbitrary thresholds.
8. Log confidence and uncertainty wherever possible.
9. Add tests for non-trivial algorithmic changes.
10. Keep hardware-specific code isolated from autonomy logic.

## SIH focus

The project must demonstrate:

- traversable terrain reasoning
- uncertainty awareness
- negative obstacle handling
- vehicle-aware risk
- dynamic obstacle avoidance
- safe stop
- recovery behavior

The final system must be demonstrable in simulation and,
where practical, on a physical UGV.
