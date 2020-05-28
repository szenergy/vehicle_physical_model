# vehicle_physical_model
Physical model for vehicles following simple kinematic rules (e.g. Kinematic and Dynamic Vehicle Models for Autonomous Driving Control Design).

__TODO__: do some basic integration to accumulate between timepoints.


# Getting started
## Installation

Clone this repository into your workspace (e.g. catkin_ws/src).
```bash
git clone https://github.com/szenergy/vehicle_physical_model
```
The requisite of the package is currently:
- autoware_msgs
- hotaru_planner (due to its statemachine framework, to synchronize incoming messages). You can install it by cloning to your workspace: ``` git clone https://github.com/kyberszittya/hotaru_planner.git -b feature/hybrid_automata```

After ensuring that all prerequisites are downloaded, build the workspace:
```
catkin build
```

## Architecture
The vehicle model can vary, but it has the following interface to ROS:
- Inputs:
  - __vehicle_status__: status of the vehicle (steer angle, linear velocity)
  - __gnss_pose__: pose supported by a GNSS sensor. Odometry is capable to initialize the initial pose based on GNSS.
- Outputs:
  - __vehicle/odom__: calculated odometry based on a vehicle model and input topics.
  - __vehicle/current_velocity__: calculated velocity based on vehicle model

The vehicle model requires the following parameters:
- __wheelbase__: vehicle wheelbase.
- __track_width__: vehicle track width.

## Launch node
At the very least, launch the node by calling _rosrun_:
```bash
rosrun szenergy_kinematic_model szenergy_kinematic_model_node
```


# Summary of commands
```bash
git clone https://github.com/szenergy/vehicle_physical_model
# Clone hotaru_planner (rei)
git clone https://github.com/kyberszittya/hotaru_planner.git -b /feature/hybrid_automata
# Build workspace
catkin build
# Start node
rosrun szenergy_kinematic_model szenergy_kinematic_model_node
```
