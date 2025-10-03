# Setup
```
git clone git@github.com:mamariomiamo/shiviz_px4_navigation.git && git submodule update --init --recursive
```

# Local Deployment (Docker)
For easy local development and testing, use the provided Docker deployment script. This builds a containerized environment with all dependencies (ROS, eCAL, MAVROS, etc.) pre-installed.

## Prerequisites
- Docker installed and running
- Git (for cloning)
- Internet connection (for downloading dependencies)

## Usage
```bash
# Build and run the container (default tag: local, drone ID: 1)
./docker/local-deploy.sh

# Or specify custom tag and drone ID
./docker/local-deploy.sh custom-tag 2
```

## What It Does
1. Builds the Docker image with ROS Noetic, eCAL, and your project code
2. Stops/removes any existing container
3. Runs a new container with host networking for ROS communication
4. Automatically cleans up dangling Docker images

## Running Your Code
Once deployed, access the container:
```bash
sudo docker exec -it shiviz_nav_local bash
```

Inside the container, source ROS and run your scripts:
```bash
source /catkin_ws/devel/setup.bash
python3 /catkin_ws/src/shiviz_px4_nav/src/odom.py
```

Or run the full ROS launch:
```bash
roslaunch shiviz_px4_nav px4_offboard.launch
```

## Stopping the Container
```bash
sudo docker stop shiviz_nav_local && sudo docker rm shiviz_nav_local
```

# Building
```
colcon build
```

## Running odom.py
This script requires a specific set of Python dependencies to bridge eCAL messages to ROS. These instructions are for a system with Python 3.8 (like Ubuntu 20.04). Make sure ros and vk180 ecal is installed before this.

```bash
# System headers & tools
sudo apt-get update
sudo apt-get install -y capnproto libcapnp-dev python3-dev build-essential

# Make sure pip is recent, then pin Cython < 3 (critical)
python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install "Cython<3"

# Install a compatible pycapnp (these build well on Py3.8/20.04)
python3 -m pip install "pycapnp==1.3.0" \
  || python3 -m pip install --no-build-isolation "pycapnp==1.3.0" \
  || python3 -m pip install --no-build-isolation "pycapnp==1.2.1"

# Verify
python3 -c "import capnp; print(capnp.__version__)"
```

# After launching PX4 gazebo sim
```
roslaunch px4_offboard px4_offboard.launch
```
# With actual drone
## Start offboard, auto-takeoff and mission using RC
Make sure RC is correctly setup as following:
### Channel 7
Channel 7 is a 3-position switch and will be used to control the state machine to go to ```TAKEOFF``` or ```MISSION``` mode.
### Channel 8
Channel 8 is a 2-position swtich and will be used to control the state machine to go to ```LAND``` or ```IDLE``` mode.
### Control logic
By default channel 7 and channel 8 should be both at upper position i.e with value 1000:
- Step 0: Set Mode Switch to POSITION Mode
- Step 1: Make sure the drone is clear from obstacles and in takeoff position.
- Step 2: Flip channel 7 switch to center position i.e. with value ~1500. Drone will automatically arm and takeoff.
- Step 3: After drone reaches takeoff height, flip channel 7 switch to bottom position i.e. with value ~1800. Drone will start to do waypoint mission.
- Step 4: After drone finishes waypoint mission, it will automatically land.
- Step 5: Flip channel 8 to bottom position.
- Step 6: Flip channel 7 to upper position.
- Step 7: Flip channel 8 to upper position to reset the state to ```IdLE```.
- Can repeat from Step 2 to trigger another round of auto-takeoff and mission.

Note:
If during mission, safety pilot can choose to flip channel 8 to bottom position to manually trigger the ```LAND``` state. Continue from Step 6 to reset the state and restart the mission.

Alternatively, safety pilot can toggle the mode switch to take over the control and manually control the drone in either POSITION or MANUAL mode.

### Radiomaster Boxer as an example
SC can be used as Channel 7 and SD can be used as Channel 8 (the two switches are at top right).
![alt text](media/boxer.png)
Make sure in the QGroundControl Flight Modes Channel Monitor, channel 7 and 8 moves when you toggle the switch.
![alt text](media/qgc.png)


### Launch command
```
roslaunch px4_offboard px4_offboard.launch use_rc:=true use_sim:=false
```
