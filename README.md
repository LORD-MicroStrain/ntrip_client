# ROS NTRIP Client

## Description

ROS node that will communicate with an NTRIP server to receive RTCM corrections and publish them on a ROS topic. Also works with virtual/relayed NTRIP servers by subscribing to NMEA
messages and sending them to the NTRIP server.

## Build Instructions

It is assumed you have already installed ROS.
Build this package from source as follows:

1. Clone this repo into the your_workspace/src directory.

2. Install rosdeps for this package: `rosdep install --from-paths ~/your_workspace/src --ignore-src -r -y`

3. Build your workspace:
    ```bash
    cd ~/your_workspace
    colcon build
    source ~/your_workspace/install/setup.bash
    ```        
    The source command may need to be run in each terminal prior to launching a ROS node.

## Connect to a NTRIP caster or server

This is useful if you have access to an NTRIP caster or server that you want to connect to over the internet.

```bash
ros2 launch ntrip_client ntrip_client_launch.py
```

-- or override defaults from cmd line (example for rtk2go) --

```bash
ros2 launch ntrip_client ntrip_client_launch.py host:=rtk2go.com mountpoint:=MyRealMtPt ntrip_server_hz:=1 authenticate:=true username:=myrealemail@provider.com password:=none
```

Launch arguments (all overridable as `name:=value`; defaults shown):

Connection:
- **host**: Hostname or IP address of the NTRIP server to connect to and receive corrections from
- **port**: Port to connect to on the server. Default: `2101`
- **mountpoint**: Mountpoint to connect to on the NTRIP server
- **ntrip_version**: Value sent in the `Ntrip-Version` request header. Default: `None` (header omitted; NTRIP rev1/ICY request)
- **user_agent**: HTTP `User-Agent` sent to the caster. **Must start with `NTRIP `.** Default: `NTRIP ponderbotics_ntrip_client`. Do not use the stock `NTRIP ntrip_client_ros` — rtk2go blocks it (see [rtk2go notes](#rtk2go-and-reconnect-behavior)).

Authentication:
- **authenticate**: Whether to authenticate with the server, or send an unauthenticated request. If `true`, `username` and `password` must be supplied.
- **username**: Username used when authenticating. For rtk2go this is your registered email. Only used if `authenticate` is true.
- **password**: Password used when authenticating. For rtk2go use `none`. Default: `none`. Only used if `authenticate` is true.

Rate & reconnect:
- **ntrip_server_hz**: Frequency (Hz) to communicate with the NTRIP server. Some servers, like rtk2go.com, will ban you if you request data too frequently — for rtk2go use `ntrip_server_hz:=1`. Default: `10`.
- **reconnect_attempt_wait_max_seconds**: Ceiling for the exponential reconnect backoff. Reconnects are persistent (the node never gives up); raise this (e.g. `:=600`) to reduce footprint during long caster outages. Default: `120` (2-minute steady-state cadence). The starting wait (`reconnect_attempt_wait_seconds`, 10s) and `rtcm_timeout_seconds` (10s) are set in the launch file's parameter block.

SSL (only used if `ssl:=true`):
- **ssl**: Connect to the caster over TLS. Default: `False`
- **cert** / **key**: Client certificate and key for cert-based auth. Default: `None`
- **ca_cert**: CA chain to use for self-signed casters. Default: `None`

Output / namespacing:
- **rtcm_message_package**: ROS message package used for published RTCM. `rtcm_msgs` (publishes `rtcm_msgs/msg/Message`) or `mavros_msgs` (publishes `mavros_msgs/msg/RTCM`). Default: `rtcm_msgs`.
- **namespace** / **group** / **node_name**: Namespace, optional sub-group, and node name. 
- **debug**: Enable debug-level logging. Default: `false`

## Connect to a NTRIP "device"

This is useful if you do not have an internet connection, but do have an NTRIP "device" that you want to receive connections from, such as the [MicroStrain 3DM-RTK](https://www.microstrain.com/inertial-sensors/3dm-rtk).

```bash
ros2 launch ntrip_client ntrip_serial_device_launch.py
```

Optional launch parameters:
- **port**: Serial port that the device is connected on. 
- **baudrate**: Baudrate to connect to the serial port at. Default 115200
- **rtcm_message_packege**: Changes the type of ROS RTCM message published by this node. Defaults to `mavros_msgs`, but also supports `rtcm_msgs`

## Topics

* **/rtcm** (publish): RTCM corrections received from the server. Message type depends on `rtcm_message_package` — `rtcm_msgs/msg/Message` by default, or `mavros_msgs/msg/RTCM`. Consumed by GNSS drivers (e.g. ublox_gps, [microstrain_inertial_driver](https://github.com/LORD-MicroStrain/microstrain_inertial)).
    * **NOTE**: The type of message can be switched between [`mavros_msgs/RTCM`](https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/msg/RTCM.msg) and [`rtcm_msgs/Message`](https://github.com/tilk/rtcm_msgs/blob/master/msg/Message.msg) using the `rtcm_message_package` parameter
* **/nmea** (subscribe): [NMEA sentence messages](http://docs.ros.org/en/api/nmea_msgs/html/msg/Sentence.html) forwarded to the NTRIP server. Needed for virtual/relayed (VRS) mountpoints. The node subscribes to the topic `/nmea`; remap it (e.g. in the launch file's `remappings`) to your NMEA source if it differs. Note: there is no `nmea_topic` launch argument — passing one has no effect.
* **/fix**: This serves the same exact purpose as `/nmea`, but facilitates receiving global position that is not in NMEA format
* **/ntrip_server_hz** (publish): A `std_msgs/String` confirmation published each communication cycle, to help verify compliance with caster usage policies.

## rtk2go and reconnect behavior

[rtk2go.com](http://rtk2go.com) runs the SNIP caster software and enforces usage policies that this fork is tuned for:

* **User-Agent blocking.** rtk2go maintains a block list of client signatures. The stock LORD-MicroStrain `User-Agent: NTRIP ntrip_client_ros` is blocked. A blocked client does **not** get a clear error — the caster returns a `SOURCETABLE 200 OK` response instead of the data stream, which the node logs as a sourcetable/invalid-response error. If you see a sourcetable response for a mountpoint you know is valid, suspect a client-side block, not a bad mountpoint. The default `user_agent` (`NTRIP ponderbotics_ntrip_client`) is accepted; if you change it, keep the mandatory `NTRIP ` prefix and avoid the stock string.
* **Request rate.** Use `ntrip_server_hz:=1` for rtk2go. Higher rates can get you banned.
* **Persistent reconnect.** On any connection loss or failed initial connect, the node schedules a non-blocking reconnect and retries indefinitely. The wait starts at `reconnect_attempt_wait_seconds` (10s) and doubles on each failure up to `reconnect_attempt_wait_max_seconds` (default 120s), then holds at that ceiling. It never gives up, so the node recovers on its own from extended rtk2go outages (DDoS) or a mountpoint taken down for maintenance. To shrink your footprint during long outages, raise the ceiling (e.g. `reconnect_attempt_wait_max_seconds:=600`).
* **First-connect timeout is normal.** With rtk2go the very first connect attempt frequently times out and then succeeds on the first backoff retry, even on healthy connections. This is expected.

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

Copyright (c)  2024, MicroStrain by HBK
