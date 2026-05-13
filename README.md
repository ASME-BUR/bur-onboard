## Building
To build all packages, run the following (from this directory):
```
scripts/build
```
Alternatively, to build a specific package:
```
scripts/build --packages-select <PACKAGE_NAME>
```
(*See ``colcon build -h`` for more options.*)

To remove all packages, run (from this directory):
```
rm -r build/ install/ log/
```

## Packages
`src/` contents:

1. `gnc/`
   - `bur_autonomy`: BehaviorTree.CPP behavior tree mission manager
   - `bur_controller`: PID controllers for translational/rotational movement
   - `bur_localization`: Parameters & launch file for EKF sensor fusion
   - `bur_thruster_manager`: Determines power for each thruster based on target force + torque
2. `launch/`
   - `bur_bringup`: Launch files for sensors & autonomous operation
   - `bur_rov`: Launch files & scripts for robot tele-operation
3. `sensors/`
   - `bur_sensors`: ROS nodes for publishing pressure sensor/IMU measurements from Raspberry Pi
   - `drivers`: sensor drivers
4. `submodules/` (created by `scripts/install_dependencies.sh`): Contains dependencies & submodules
