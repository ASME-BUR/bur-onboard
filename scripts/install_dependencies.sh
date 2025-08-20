#!/bin/bash

info () {
        echo -e "\e[32m[INFO]\e[00m " $@
}

info "Checking connection to http://github.com..."
wget -q --timeout 5 --spider http://github.com

if [ $? -eq 0 ]; then
    info "Connected, installing dependencies..."
    mkdir -p src/submodules
    vcs import --input required.repos src/submodules
else
    echo "No connection, skipping..."
fi
