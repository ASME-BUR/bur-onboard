# System #

- Ubuntu 22.04
- ROS2 Humble
- Gazebo 11

# Installation #

### 1. Install Git ###
   ```
   sudo apt update && sudo apt-get install -y git-all
   ```

### 2. Clone this repository into your root directory: ###

   ```
   cd ~/ && git clone https://github.com/ASME-BUR/bur-onboard.git && cd bur-onboard
   ```

### 3. Run the ROS installation script ###
   ```
   sudo bash scripts/setup_ros.sh
   ```
Once this command finishes, close and re-open your terminal before proceeding.

### 4. Run the repository setup script ###
  ```
  sudo bash scripts/setup_repository.sh
  ```

### 5. Install dependencies using VCS ###
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

   ```
   sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   sudo apt-get update
   sudo apt-get install gazebo11
   sudo apt-get install libgazebo11-dev
   ```
   You can write gazebo in a terminal to make sure that gazebo runs correctly. Write gazebo --version to ensure that the version number is 11.X.

### (Optional) Install the ROS packages for gazebo: ###
   ```
   sudo apt install ros-humble-gazebo-ros-pkgs
   colcon build --symlink-install
   ```
