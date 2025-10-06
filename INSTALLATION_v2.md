# System #

- Ubuntu 22.04
- ROS2 Humble
- Gazebo 11

# Installation #

### 1. Clone this repository into your root directory: ###

   ```
   cd ~/ && git clone https://github.com/ASME-BUR/bur-onboard.git && cd bur-onboard
   ```

### 2. Run the ROS installation script ###
Add the ROS2 source commands to your bashrc
   ```
   sudo bash scripts/setup_ros.sh
   ```

### 3. Run the repository setup script ###
  ```
  sudo bash scripts/setup_repository.sh
  ```

### 4. Install dependencies using VCS ###
  ```
  sudo bash scripts/install_dependencies.sh
  ```

### 5. Do a test build of BUR packages ###
  ```
  source ~/.bashrc
  scripts/build
  ```
The build script may print messages regarding "OOM SIGKILL"; this is due to an issue with `colcon` (the ROS2 build package). As long as the script continues, this is okay.

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
