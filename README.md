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

# Function for building docker ws - Place in ~/.bashrc
build_ws() {
    local branch="${1:-main}"
    echo "Cloning Branch: $branch"
    docker build -t docker_ws --build-arg CACHE_BUST=$(date +%s) --build-arg branch="$branch" --progress=plain .
}

# Working with the docker
 . run_docker.bash
    roscore
    
 . exec_docker.bash
    do ros things
