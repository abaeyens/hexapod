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
This project uses only the former.


## Naming Gazebo vs Gazebo Ignition
The Gazebo simulator is currently in the process
of dropping the name "Ignition" in favor of just "Gazebo",
and while some ROS-side packages have already adapted themselves,
Gazebo Fortress still uses "Ignition".
Where possible, the new naming has already been applied.


## GPU acceleration
### Intel
On the host, install the package `intel-gpu-tools`, then run [`intel_gpu_top`](
  https://manpages.ubuntu.com/manpages/trusty/man1/intel_gpu_top.1.html).
Then, compare the clock speeds and BUSY percentages for Gazebo running and not running.
The difference should be large. Example:

_Gazebo not running_: 91 MHz, 8 % BUSY
```
intel-gpu-top -   84/  91 MHz;   82% RC6;  0.07 Watts;      426 irqs/s

      IMC reads:     1907 MiB/s
     IMC writes:      148 MiB/s

          ENGINE      BUSY                                                                                                MI_SEMA MI_WAIT
     Render/3D/0    8.03% |███████▍                                                                                     |      0%      0%
       Blitter/0    0.00% |                                                                                             |      0%      0%
         Video/0    0.00% |                                                                                             |      0%      0%
  VideoEnhance/0    0.00% |                                                                                             |      0%      0%
```
_Gazebo running_: 628 MHz, 77 % BUSY
```
intel-gpu-top -  613/ 628 MHz;   15% RC6;  2.34 Watts;     1094 irqs/s

      IMC reads:    11798 MiB/s
     IMC writes:     5988 MiB/s

          ENGINE      BUSY                                                                                                MI_SEMA MI_WAIT
     Render/3D/0   76.96% |███████████████████████████████████████████████████████████████████████▌                     |      0%      0%
       Blitter/0    0.00% |                                                                                             |      0%      0%
         Video/0    0.00% |                                                                                             |      0%      0%
  VideoEnhance/0    0.00% |                                                                                             |      0%      0%
```

### Nvidia
To use the Nvidia GPU, uncommenting the line
`runtime: nvidia` in `docker-compose.yml` tends to help
(but is not always sufficient).
Nvidia GPU usage can be checked directly
using `nvtop` utility (package `nvtop`),
and indirectly by verifying that the usage
of the Intel (or other integrated) GPU remains low.
Gazebo is very CPU heavy and as a result, when a discrete (Nvidia) GPU is used,
the CPU tends to become the bottleneck and the GPU usage
will not increase as much as in the integrated case.

[Notes on installing the Nvidia container-toolkit](
  https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)
for using Nvidia GPUs inside containers.
