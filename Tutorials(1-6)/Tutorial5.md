## Tutorial 5: Installing ROS and Setting Up Catkin Workspace

### What is ROS?

**ROS (Robot Operating System)** is not actually an operating system, but a middleware framework for robotics development.

**What ROS Provides:**
- **Message Passing System** - Different programs communicate via "topics" (think of it as a pub/sub messaging system)
- **Hardware Abstraction** - Write code once, use on different hardware
- **Package Management** - Organize code into reusable modules
- **Tools & Libraries** - Path planning, computer vision, sensor processing already built
- **Community Packages** - Thousands of open-source packages for various sensors and algorithms

**Why We Use ROS for Drones:**
Without ROS, you'd have to write low-level code to:
- Read sensor data from multiple sources
- Communicate between different programs
- Coordinate between vision processing and flight control
- Handle message synchronization and timing

ROS does all this for you, letting you focus on **autonomy logic** instead of plumbing code.

**Analogy:**
- **Without ROS:** You're building both the car and the road
- **With ROS:** Someone already built the road, you just drive

### What is Catkin?

**Catkin** is ROS's build system - it compiles your code and manages dependencies between packages.

Think of it like:
- `make` for C/C++ projects
- `npm` for Node.js projects
- `pip` for Python projects

But specifically designed for ROS packages that might mix Python, C++, and configuration files.

### What are MAVROS and MAVLink?

**MAVLink:**
- Lightweight **communication protocol** designed for drones
- Defines message formats for telemetry, commands, mission data
- Used by ArduPilot, PX4, and most flight controllers
- Optimized for low-bandwidth radio links

**MAVROS:**
- **MAVLink + ROS bridge**
- Converts MAVLink messages into ROS messages
- Allows your ROS code to communicate with ArduPilot/PX4
- Without MAVROS, you'd have to parse MAVLink binary messages manually

**The Communication Chain:**
```
Your Python Script → ROS Topic → MAVROS → MAVLink → ArduPilot → Drone Motors
```

### Installation for Ubuntu 20.04 + ROS Noetic

#### Step 1: Setup ROS Repository
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### Step 2: Add ROS Keys
```bash
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

#### Step 3: Update and Install ROS Noetic
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full -y
```

#### Step 4: Initialize rosdep
```bash
sudo rosdep init
rosdep update
```

#### Step 5: Setup Environment
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Step 6: Install Build Dependencies
```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install python3-catkin-tools python3-osrf-pycommon -y
```

### Setting Up Catkin Workspace

#### Step 1: Create Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

#### Step 2: Build Workspace
```bash
catkin build
```

#### Step 3: Source Workspace
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Installing MAVROS and MAVLink

#### Step 1: Initialize wstool
```bash
cd ~/catkin_ws
wstool init ~/catkin_ws/src
```

#### Step 2: Get MAVROS and MAVLink Source
```bash
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
```

#### Step 3: Install Dependencies
```bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
```

#### Step 4: Build MAVROS
```bash
catkin build
source ~/.bashrc
```

#### Step 5: Install GeographicLib Dataset
```bash
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

### Installing IQ Simulation Package

#### Step 1: Clone Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/Intelligent-Quads/iq_sim.git
```

#### Step 2: Add Gazebo Models Path
```bash
echo "export GAZEBO_MODEL_PATH=\${GAZEBO_MODEL_PATH}:\$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc
source ~/.bashrc
```

#### Step 3: Install ROS-Gazebo Bridge
```bash
sudo apt install ros-noetic-gazebo-ros ros-noetic-gazebo-plugins -y
```

#### Step 4: Build Workspace
```bash
cd ~/catkin_ws
catkin build
source ~/.bashrc
```

#### Step 5: Create SITL Launch Script
```bash
cp ~/catkin_ws/src/iq_sim/scripts/startsitl.sh ~/
chmod +x ~/startsitl.sh
```

---