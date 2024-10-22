# fc_planner
Rep for docker image of fc_planner


# Get docker up and running:
sudo apt get install docker.io

sudo groupadd docker

sudo usermod -aG docker $USER

Restart PC

docker pull osrf/ros:noetic-desktop-full

#cd into the cloned ws
docker build -t docker_ws.

# Alias for docker build
alias build_ws="cd fc_planner && docker build -t docker_ws ."

# Working with the docker
 . run_docker.bash
    roscore
    
 . exec_docker.bash
    do ros things
