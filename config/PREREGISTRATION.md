# Robot-arm planning/execution benchmark: simulation only

This protocol is frozen before evaluation. Development uses seeds 1000-1099, evaluation seeds 0-19. Do not revise scenes/budgets after inspecting held-out performance.

Compare MoveIt OMPL RRTConnect and PRM from fresh planner contexts, identical start/goal configurations and obstacle geometry. Ten scenes, twenty independent runs per planner = 400 attempts. Planning budget 1.0 wall second, one planning attempt, single thread. Record seed, actual machine/software, scene/model/config hashes, status, wall planning time and solver-reported time separately. Reject invalid start/goal, unreachable out-of-bounds goals and collision paths rather than reporting them as successes. Include collision-goal and narrow-passage cases, even if every attempt fails.

Report success/timeout/invalid-start/invalid-goal/collision-validity explicitly. Timing median/p95 is computed separately for all attempts and successful plans; no mixture with execution runtime. Joint-space path length = sum Euclidean waypoint differences (radians). Dense collision/bounds checking interpolates at maximum joint step .01 rad; finite resolution is a stated limitation, not a continuous collision guarantee.

Execute each valid plan in Gazebo Classic dynamics with torque-limited PD joint control, same original procedural 6DOF model and inertial parameters. No kinematic teleportation during execution. Initial joint positioning before execution is allowed and recorded; interpolate the commanded trajectory at <=0.5 rad/s. Simulated joint tracking RMSE/p95/max is measured against that command from actual simulated joint state, separately from planner timing. Report divergence, contact and tracking failure. This is not ROS2 hardware control or real-arm accuracy.

Hypothesis: neither planner is declared best in advance. Compare distributions and failure cases; no significance claim from a small fixed scene set. Demo uses a predeclared first successful case in scene order, both planner outcomes and full failure table retained. Model is original procedural geometry, not a branded robot.

## Development corrections before held-out evaluation

Development seeds 1000-1002 exposed unstable dynamics from tiny wrist inertias and explicit joint damping. The final original model uses a .005 kg m2 minimum for each principal inertia and .005 joint damping; this is synthetic, not identified from hardware. PD gains are fixed at kp=[40,40,30,10,8,5], kd=[3,3,2,.6,.5,.3], torque limit 30 Nm, Gazebo step .001 s. Initial settle/terminal hold are .5 s each and included in pooled tracking metrics. Non-finite position, velocity or torque and velocity >100 rad/s stop execution. Earlier failed development traces remain available; they are not valid evaluation evidence. No held-out runs occurred before these corrections.

Execution completion only means the simulation ran to its end. Report tracking pass separately as RMSE <=.15 rad, with bounds/collision checks on sampled actual states every .01 simulated s. This finite-rate check is not continuous safety. It does not erase an unsafe tracking trace or make it real hardware accuracy. Planning failures discovered by dense revalidation are retained as invalid_path, not silently fixed.
