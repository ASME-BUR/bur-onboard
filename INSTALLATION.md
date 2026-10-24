# System #

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic

# Installation #

### 1. Install Git ###
   ```
   sudo apt update && sudo apt-get install -y git-all
   ```

### 2. Clone this repository into your root directory: ###

   ```
   cd ~/ && git clone https://github.com/ASME-BUR/bur-onboard.git && cd bur-onboard
   ```

### 3. Run our ROS installation script ###
   ```
   sudo bash scripts/setup_ros.sh
   ```
(This will likely take ~8-12 minutes.)

### 4. Add sources to ~/.bashrc ###
   ```
   grep -F "source /opt/ros/jazzy/setup.bash" ~/.bashrc \
       || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc \
       || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
   grep -F "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" ~/.bashrc \
       || echo "# export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc
   ```
After running this command, close and re-open your terminal before proceeding.

(Make sure to `cd` into the `bur-onboard` repository folder again before proceeding.)

### 4. Run our repository setup script ###
  ```
  sudo bash scripts/setup_repository.sh
  ```

### 5. Run our VCS dependency installation script ###
  ```
  sudo bash scripts/install_dependencies.sh
  ```

### 6. Do a test build of BUR packages ###
  ```
  source ~/.bashrc
  scripts/build
  ```
The build script may print messages regarding "OOM SIGKILL"; this is due to an issue with `colcon` (the ROS2 build package). As long as the script continues, this is okay.

### 7. Test your installation ###
To verify that your install and build succeeded, feel free to run the following command:
```
ros2 launch bur_bringup rov.launch.py
```
If you run ```ros2 node list``` (in a separate terminal), you should see something similar to the following:
```
root@d53438d5ffdb:/# ros2 node list
/bur_controller
/joy
/joy_command
/thruster_manager
```
(Your results may look slightly different, depending on your version of the repository; as long as some nodes appear, this is OK.)

### (Optional) Install Gazebo: ###
Gazebo Classic (Gazebo 11) is not supported on Ubuntu 24.04 / ROS2 Jazzy. Jazzy pairs with Gazebo Harmonic, which is installed together with the ROS bridge packages:
   ```
   sudo apt-get update
   sudo apt-get install ros-jazzy-ros-gz
   ```
   Run `gz sim` in a terminal to make sure that Gazebo runs correctly, and `gz sim --version` to ensure that the version number is 8.X (Harmonic).
