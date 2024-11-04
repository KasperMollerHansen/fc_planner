# fc_planner
Rep for docker image of fc_planner


# Get docker up and running:
```
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo groupadd docker

sudo usermod -aG docker $USER
```
Restart PC
```
docker pull osrf/ros:noetic-desktop-full

#cd into the cloned ws
docker build -t docker_ws.
```
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
