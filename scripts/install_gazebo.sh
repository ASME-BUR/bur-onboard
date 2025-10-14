#!/bin/bash

info () {
        echo -e "\e[32m[INFO]\e[00m " $@
}

info "Installing Gazebo..."

sudo apt-get update
sudo apt-get install lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress

info "Cloning bur-gazebo repository..."
info "Checking connection to http://github.com..."
wget -q --timeout 5 --spider http://github.com

if [ $? -eq 0 ]; then
    info "Connected, installing dependencies..."
    mkdir -p src/submodules
    vcs import --input gazebo.repos src/submodules
else
    info "No connection, skipping..."
fi

rosdep install --from-paths src --ignore-src -r -y
