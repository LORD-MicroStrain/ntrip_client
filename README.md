# ROS NTRIP Client

## Description

ROS node that will communicate with an NTRIP server to receive RTCM connections and publish them on a ROS topic. Also works with network/VRS mountpoints by subscribing to NMEA
messages and sending them to the NTRIP server

#### Important Branches
There are two important branches that you may want to checkout:

* [ros](https://github.com/LORD-MicroStrain/ntrip_client/tree/ros) -- Contains ROS1 implementation for this node.
* [ros2](https://github.com/LORD-MicroStrain/ntrip_client/tree/ros2) -- Contains ROS2 implementation for this node.

## Build Instructions

#### Building from source
1. Install ROS and create a workspace: [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

2. Move the entire ntrip_client folder to the your_workspace/src directory.

3. Install rosdeps for this package: `rosdep install --from-paths ~/your_workspace/src --ignore-src -r -y`

4. Build your workspace:
    ```bash
    cd ~/your_workspace
    catkin_make
    source ~/your_workspace/devel/setup.bash
    ```        
    The source command may need to be run in each terminal prior to launching a ROS node.

#### Launch the node and publish data
The following command will launch the node. Keep in mind each instance needs to be run in a separate terminal.
```bash
roslaunch ntrip_client ntrip_client.launch
```

Optional launch parameters:
- **host**: Hostname or IP address of the NTRIP server to connect to.
- **port**: Port to connect to on the server. Default: `2101`
- **mountpoint**: Mountpoint to connect to on the NTRIP server.
- **authenticate**: Whether to authenticate with the server, or send an unauthenticated request. If set to true, `username`, and `password` must be supplied.
- **username**: Username to use when authenticating with the NTRIP server. Only used if `authenticate` is true
- **password**: Password to use when authenticating with the NTRIP server. Only used if `authenticate` is true

#### Topics

This node currently only has two topics of interest:

* **/rtcm**: This node will publish the RTCM corrections received from the server to this topic as [RTCM messages](http://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/RTCM.html). These messages can be consumed by nodes such as the [microstrain_inertial_driver](https://github.com/LORD-MicroStrain/microstrain_inertial)
* **/nmea**: This node will subscribe on this topic and receive [NMEA sentence messages](http://docs.ros.org/en/api/nmea_msgs/html/msg/Sentence.html) which it will forward to the NTRIP server. This is only needed when using a virtual NTRIP server

## Docker Integration

### VSCode

The easiest way to use docker while still using an IDE is to use VSCode as an IDE. Follow the steps below to develop on this repo in a docker container

1. Install the following dependencies:
    1. [VSCode](https://code.visualstudio.com/)
    1. [Docker](https://docs.docker.com/get-docker/)
1. Open VSCode and install the following [plugins](https://code.visualstudio.com/docs/editor/extension-marketplace):
    1. [VSCode Remote Containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
1. Open this directory in a container by following [this guide](https://code.visualstudio.com/docs/remote/containers#_quick-start-open-an-existing-folder-in-a-container)

### Make

If you are comfortable working from the command line, the [Makefile](./devcontainer/Makefile) in the [.devcontainer](./devcontainer) directory
can be used to build a development image, and run a shell inside the docker image. Follow the steps below to setup your environment to use the `Makefile`

1. Install the following dependencies:
    1. [Make](https://www.gnu.org/software/make/)
    1. [Docker](https://docs.docker.com/get-docker/)
    1. [qemu-user-static](https://packages.ubuntu.com/bionic/qemu-user-static) (for multiarch builds)
        1. Run the following command to register the qemu binaries with docker: `docker run --rm --privileged multiarch/qemu-user-static:register`

The `Makefile` exposes the following tasks. They can all be run from the `.devcontainer` directory:
* `make build-shell` - Builds the docker image and starts a shell session in the image allowing the user to develop and build the ROS project using common commands such as `catkin_make`
* `make clean` - Cleans up after the above two tasks

## License
ntrip_client is released under the MIT License - see the `LICENSE` file in the source distribution.

Copyright (c)  2021, Parker Hannifin Corp.
