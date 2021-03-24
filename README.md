# M-A-R-I-A Robot Simulation

## Setup
There are two ways of developing for this project: docker container and normal ROS install

### [Native] Normal ROS Install setup:
Install ROS Melodic normally: http://wiki.ros.org/melodic/Installation

### [Docker] Developing with Docker & VS Code with DevContainers

Once prerequisites are installed and set up, all you have to do is open the VS Code command window (`ctrl+shift+p` or `cmd+shift+p`) and type `Remote-Container: Rebuild and Reopen in Container`, or click on the green button on the bottom left and select the same command. To exit container, simply hit the green button on the bottom right and click `Remote-Container: Reopen Locally`

The `Dockerfile` and `docker-compose.yml` are all contained in the `.devcontainer` directory, which also contains the devcontainer setup for VS Code. In order to attach a VS Code instance inside the container, you must install the Remote Development Extension in VS Code: `ms-vscode-remote.vscode-remote-extensionpack`

Make sure you also install Docker: https://docs.docker.com/engine/install/ubuntu/

You will also need Docker-Compose: https://docs.docker.com/compose/install/

Finally, you will need nVidia Docker: https://github.com/NVIDIA/nvidia-docker

You will also need to set up your ssh-add agent to automatically pass your ssh keys to the devcontainer. Follow the instructions here: https://code.visualstudio.com/docs/remote/containers#_sharing-git-credentials-with-your-container

### [Native & Docker] build environment + dependencies
Once you are in your environment (and have downloaded the repo), it's time to `catkin_make` the project.
1. Update the ROS dependencies list:
```bash
rosdep update
```
2. Install ROS Dependencies:
```bash
sudo apt update
sudo rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```