# FASTER: Fast and Safe Trajectory Planner for Flights in Unknown Environments

## About
This is adapted from original [repo](https://github.com/mit-acl/faster), by MIT-acl(Aerospace Controls Lab). Original repo demonstrates capabilities:
- quadrotor control
- simulate depth camera
- 3D point cloud from depth image(considering each pixel as point)
- notion of unknown space, free and occupied space
- real-time planning to reach goal specified solely by relative coordinates in unknown map
- visualization of trajectory, (space/free/occ) grids, gazebo physics simulation.

with limitations:
- Controller is useless as it assumes perfect tracking with unbounded actuation. Even take-off uses pseudo p-controller to set altitude iteratively.
- Unknown and occupancy grid are of world dimensions. Memory costly.
- for ellipse inflation in safe flight corridor, all points are checked iteratively. points clipped to cube vertices. time costly
- uses perfect state directly from simulation
- assumption of depth camera

Goal of this repo is to build FASTER, its dependencies and ros from source on linux(Fedora because Ubuntu sucks), so that my own UAV and perception/planning algorithms can be plugged-in gradually and simulated. Another goal is to possibly port to ROS2.

I have included `src` directory containing dependencies too, to include changes on pulled sub-repos. Git submodules does not works as submodules needs to be pushed to corresponding repos. Patch file may have been better option but, I have no previous experience with it.

## Instructions
Tested on:
- ROS-noetic
- Fedora OS
- Macbook Pro (2012)

But, you should be able to adapt, as scarcely depends on pre-built binaries.
### Pull dependencies
```
./fetch.sh
```
### Build
```
./build.sh
```
### Launch simulation
```
./run.sh
```
### Interact
* High level command to start quadrotor and hover
    * `rqt: Plugins > Robot Tools > Mission Mode > [click] START`
* Set goal to nvigate to: 
    * `rviz: Tools(panel) > 2D Nav Goal > [click at any goal location]`
* View Unknown space grid: 
    * `rviz: Displays(panel) > [Check] UnkownGrid`
