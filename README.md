# ArmBench

Work in progress. A six-DOF arm planning and execution benchmark using ROS2, MoveIt OMPL and Gazebo Classic dynamics. Simulation only; no hardware or real-arm accuracy claim.

Held-out protocol: 10 fixed scenes, 20 seeds each, RRTConnect vs PRM, 1-second planner budgets. Invalid/unreachable goals, blocked starts, narrow passages and dense-invalid paths are kept. Planner timing is separate from simulated torque-control execution and actual joint-state tracking.

Current raw results are under `results/evaluation`. Development traces are separate and include failed controller trials, not evaluation evidence. Full report/reproduction/tests/demo/CI are being finished; this is not the final release.

Original procedural model, synthetic inertias. MIT code; ROS2/MoveIt/Gazebo/OMPL and other dependencies retain their own licenses. Exact Linux conda distribution is pinned in `conda-linux-64.lock`.
