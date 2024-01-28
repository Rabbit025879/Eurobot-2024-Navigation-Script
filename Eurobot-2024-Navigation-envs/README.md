# Navigation ROS Noetic Guide

This repository setup the environment Eurobot2024 for you. 

> If you want to know what's going on inside the Dockerfile, devcontainer.json, and docker-compose.yaml, please read [Dokcer Quick Guide](./docs/docker_quick_guide.md).

## Setup
### 1. First-time setup
If this is your first time running container on your machine, please follow the steps below. Otherwise, you can jump to [Customized Setup](#2-customized-setup).
```
sudo apt install docker
sudo apt install docker-compose
```

## 2. Customized Setup
Search for word "IMPORTANT" in Dockerfile and docker-compose.yaml

<b>In Dockerfile</b>
```
# [IMPORTANT 1/3] Change USERID to your user id
ARG USERID=1003

# Ex: for local, it might be 1000,
#     for R2-2, it might be 1003 (for user Navigation)
```

<b>In docker-compose.yaml</b>
```
volumes:
    # [IMPORTANT 2/3] Change the path to the navigation workspace on your host machine
    - ${HOME}/navigation/Eurobot2024-machine-ws/:/home/user/Eurobot2024-Navigation-ws

# [IMPORTANT 3/3] Change container hostname to that of the host machine's 
    hostname: cleaner # Ex: DIT-2024-02
```


## 3. Build the Container
```
# Enter one env folder 
cd Eurobot2024-Navigation-envs/navigation-noetic-R2-2

# build the docker image
docker-compose up --build

# then use your docker pluing in vscode to attach devcontainer to VScode.
```