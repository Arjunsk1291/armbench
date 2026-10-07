# Third-party notices

Original ArmBench model, scene definitions and code: MIT, Copyright 2026 Arjun Sunil Kumar. No external robot meshes or dataset copied. Synthetic model dimensions/inertias do not represent a commercial arm.

- ROS2 `rclcpp`: Apache-2.0. `urdf` and `pluginlib`: BSD, as declared by installed metadata.
- MoveIt Core and OMPL plugin, SRDFDOM: BSD, as declared by installed package metadata. Source/header notices are preserved under `docs/licenses`.
- OMPL: BSD-3-Clause. https://ompl.kavrakilab.org/license.html
- Gazebo Classic: Apache-2.0. https://github.com/gazebosim/gazebo-classic
- RoboStack binary distribution: dependencies retain upstream licenses. The exact package URLs/builds are recorded in `conda-linux-64.lock`; package-level metadata is in `docs/licenses/package_metadata.json`. This lock does not relicense packages.
- NumPy/SciPy/imageio: their BSD licenses; Matplotlib: its own license; Pillow: MIT-CMU. Report dependencies are pinned in `report-requirements.lock`. Copies of their installed texts are in `docs/licenses/report_dependencies`.
- FFmpeg used by imageio-ffmpeg to encode the demo has its own build/license, recorded in `docs/licenses/report_dependencies/ffmpeg_build_license.txt`. Encoded output is a forward-kinematics visualization of our own Gazebo traces, not a redistributed robot asset.

Source references: https://github.com/moveit/moveit2, https://github.com/ros2/rclcpp, https://github.com/ompl/ompl, https://robostack.github.io/conda.html.
