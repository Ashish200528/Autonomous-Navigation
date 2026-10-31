# Phase 1 evidence

These files are command output captured from the WSL environment. Package versions and post-install resources are summarized in post-install-verification.log.

- ros-talker.log and ros-listener.log: actual ROS demo message output. ros-talker-listener-result.txt records message counts and stop status.
- gazebo-headless.log: Gazebo Harmonic server output for the installed empty world. The 12-second runtime bound was intentional; the command wrapper did not retain the child's timeout exit code, so the pass is based on the logged world initialization and advertised services.
- rviz2.log: RViz2 WSLg startup messages and OpenGL version. A screenshot was inspected in the setup conversation but not saved as an image file. The runtime-directory warning is included.
- nav2-headless.log: first bounded TurtleBot3/Nav2 sample launch. It includes a successful robot entity creation and startup-time transform messages.
- nav2-diagnostic-launch.log and nav2-diagnostic-graph.log: follow-up headless launch and direct live topic probes. The probes received clock, odometry, transform, and laser-scan messages. No initial pose or navigation goal was sent.

Log line endings and terminal padding were normalized for readable diffs; message text and reported values were preserved.
