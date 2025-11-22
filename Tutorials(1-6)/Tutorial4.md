## Tutorial 4: Installing Gazebo and ArduPilot Plugin

### Why Gazebo?
Gazebo provides 3D visualization, physics simulation, and simulated sensors (camera, LiDAR, sonar) for realistic testing.

**What is Gazebo?**
Gazebo is an open-source robotics simulator originally developed for DARPA robotics challenges. It's **not** drone-specific - it's used for all types of robots (ground robots, manipulators, underwater vehicles, etc.).

**Why We Need a Plugin for ArduPilot:**
- Gazebo is a **general-purpose robotics simulator**
- ArduPilot is **drone-specific flight control software**
- The **ArduPilot-Gazebo plugin** acts as a bridge, allowing them to communicate:
  - Gazebo simulates physics and sensors → sends data to ArduPilot
  - ArduPilot calculates motor commands → sends to Gazebo
  - This creates a closed-loop simulation

**What Gazebo Adds to Our Setup:**
- ✅ **3D Visualization** - See your drone flying in a virtual world
- ✅ **Physics Engine** - Realistic gravity, aerodynamics, collisions
- ✅ **Sensor Simulation** - Virtual cameras, LiDAR, GPS, IMU with realistic noise
- ✅ **Environment Testing** - Add obstacles, buildings, moving objects
- ✅ **Multi-drone Simulation** - Test swarm algorithms

**The Complete Stack So Far:**
```
┌─────────────────────┐
│   Your Code         │  (Python/C++ autonomy scripts)
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│   MAVROS/ROS        │  (Message passing, not yet installed)
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│   ArduPilot SITL    │  (Flight control firmware)
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│   Gazebo Simulator  │  (Physics, sensors, 3D world)
└─────────────────────┘
```

### Installation for Ubuntu 18.04

#### Step 1: Add Gazebo Repository
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

#### Step 2: Add Repository Key
```bash
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

#### Step 3: Update Package List
```bash
sudo apt update
```

#### Step 4: Install Gazebo 9
```bash
sudo apt install gazebo9 libgazebo9-dev -y
```

### Installation for Ubuntu 20.04

```bash
# Steps 1-3 same as above
sudo apt install gazebo11 libgazebo11-dev -y
```

### Installing ArduPilot Gazebo Plugin

#### Step 1: Clone the Plugin Repository
```bash
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
```

#### Step 2: Checkout Dev Branch (Ubuntu 18.04 only)
```bash
git checkout dev
```

#### Step 3: Build the Plugin
```bash
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

#### Step 4: Configure Environment Variables
```bash
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
source ~/.bashrc
```

### Running Gazebo with ArduPilot

#### Terminal 1: Launch Gazebo World
```bash
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

#### Terminal 2: Launch SITL
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

### Testing in Gazebo

In the SITL terminal (MAVProxy), run:
```bash
mode guided
arm throttle
takeoff 2
```

You should see the drone take off in Gazebo!

---