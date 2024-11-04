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
```
build_ws() {
    local branch="${1:-main}"
    echo "Cloning Branch: $branch"
    docker build -t docker_ws --build-arg CACHE_BUST=$(date +%s) --build-arg branch="$branch" --progress=plain .
}
```

# Working with the docker
 . run_docker.bash
    roscore
    
 . exec_docker.bash
    do ros things

# Credit to:
* [FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes](https://ieeexplore.ieee.org/document/10610621), Chen Feng, Haojia Li, Mingjie Zhang, Xinyi Chen, Boyu Zhou, and Shaojie Shen, 2024 IEEE International Conference on Robotics and Automation (ICRA).

```
@inproceedings{feng2024fc,
  title={Fc-planner: A skeleton-guided planning framework for fast aerial coverage of complex 3d scenes},
  author={Feng, Chen and Li, Haojia and Zhang, Mingjie and Chen, Xinyi and Zhou, Boyu and Shen, Shaojie},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={8686--8692},
  year={2024},
  organization={IEEE}
}
```
