# Robot-arm planning/execution benchmark: simulation only

This protocol is frozen before evaluation. Development uses seeds 1000-1099, evaluation seeds 0-19. Do not revise scenes/budgets after inspecting held-out performance.

Compare MoveIt OMPL RRTConnect and PRM from fresh planner contexts, identical start/goal configurations and obstacle geometry. Ten scenes, twenty independent runs per planner = 400 attempts. Planning budget 1.0 wall second, one planning attempt, single thread. Record seed, actual machine/software, scene/model/config hashes, status, wall planning time and solver-reported time separately. Reject invalid start/goal, unreachable out-of-bounds goals and collision paths rather than reporting them as successes. Include collision-goal and narrow-passage cases, even if every attempt fails.

Report success/timeout/invalid-start/invalid-goal/collision-validity explicitly. Timing median/p95 is computed separately for all attempts and successful plans; no mixture with execution runtime. Joint-space path length = sum Euclidean waypoint differences (radians). Dense collision/bounds checking interpolates at maximum joint step .01 rad; finite resolution is a stated limitation, not a continuous collision guarantee.

Execute each valid plan in Gazebo Classic dynamics with torque-limited PD joint control, same original procedural 6DOF model and inertial parameters. No kinematic teleportation during execution. Initial joint positioning before execution is allowed and recorded; interpolate the commanded trajectory at <=0.5 rad/s. Simulated joint tracking RMSE/p95/max is measured against that command from actual simulated joint state, separately from planner timing. Report divergence, contact and tracking failure. This is not ROS2 hardware control or real-arm accuracy.

Hypothesis: neither planner is declared best in advance. Compare distributions and failure cases; no significance claim from a small fixed scene set. Demo uses a predeclared first successful case in scene order, both planner outcomes and full failure table retained. Model is original procedural geometry, not a branded robot.
