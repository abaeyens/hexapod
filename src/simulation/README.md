# Simulation
This packages handles everything strictly required for simulation.
It is therefore not required for running the real-life robot.

## ROS <=> Gazebo interface
There are two ways to communicate between ROS and Gazebo.
A first is using `ros_gz_bridge`. In that case, the bridge
does the conversion between ROS and Gazebo topics.
A second is using an embedded node, i.e. custom code
that is run as part of the simulation, which just interacts with ROS.
For more: see the video
[ROS 2 and Gazebo Integration Best Practices.mp4](
  https://vimeo.com/showcase/9954564/video/767127300).

This project uses both:
- Information flow from Gazebo to ROS mostly uses `ros_gz_bridge`.
  This is configured in `bridge.yaml`.
- Information flow from ROS to Gazebo mostly uses embedded nodes,
  which is configured in those nodes themselves.

## Naming Gazebo vs Gazebo Ignition
The Gazebo simulator is currently in the process
of dropping the name "Ignition" in favor of just "Gazebo",
and while some ROS-side packages have already adapted themselves,
Gazebo Fortress still uses "Ignition".
Where possible, the new naming has already been applied.
