## Tutorial 6: ROS Basics, Topics, and Accessing Drone Telemetry

### Understanding ROS Topics

**What is a ROS Topic?**
A topic is like a radio channel - programs can "publish" (broadcast) messages on a topic, and other programs can "subscribe" (listen) to that topic.

**Analogy:**
- **YouTube channel** = ROS topic
- **YouTuber uploading Tutorials** = Publisher
- **Subscribers watching** = Subscribers
- Multiple people can watch (subscribe) to the same channel
- Multiple channels (topics) can exist simultaneously

**Why This Matters for Drones:**
Your drone system has many components that need to share data:
```
Camera Node → /webcam/image_raw → Your Vision Processing Node
Gazebo → /gazebo/model_states → Your Position Tracker
ArduPilot → /mavros/state → Your Mission Planner
Your Code → /mavros/setpoint_position/local → ArduPilot
```

### The IQ Sim Package

**What is iq_sim?**
A pre-configured ROS package that includes:
- Custom Gazebo worlds (runway, obstacles, etc.)
- Launch files to start everything with one command
- Helper scripts for ArduPilot SITL
- Example drone models with sensors

**Why Use It?**
Instead of manually starting 3-4 terminals with complex commands, you can launch the entire simulation stack with one command. It's a teaching tool that simplifies the setup.

### Launching the Complete Simulation

#### Terminal 1: Launch Gazebo World with ROS
```bash
roslaunch iq_sim runway.launch
```

#### Terminal 2: Launch SITL
```bash
~/startsitl.sh
```

#### Terminal 3: Launch MAVROS
```bash
roslaunch iq_sim apm.launch
```

### Understanding ROS Topics

#### List All Active Topics
```bash
rostopic list
```

#### View Data from a Topic (Live Feed)
```bash
# See true position from Gazebo simulation
rostopic echo /gazebo/model_states

# See estimated drone position (what we use for autonomy)
rostopic echo /mavros/global_position/local

# See drone state (armed, mode, connected)
rostopic echo /mavros/state

# See battery status
rostopic echo /mavros/battery

# See IMU data
rostopic echo /mavros/imu/data
```

#### Get Topic Information
```bash
# Show message type
rostopic info /mavros/global_position/local

# Show publishing/subscribing rate
rostopic hz /mavros/global_position/local

# Show message structure
rostopic type /mavros/global_position/local | rosmsg show
```

#### Inspect Message Types
```bash
# Show structure of Odometry message
rosmsg show nav_msgs/Odometry

# Show structure of State message
rosmsg show mavros_msgs/State

# Show structure of PoseStamped message
rosmsg show geometry_msgs/PoseStamped
```

### Key MAVROS Topics Reference

**Understanding the Topic Names:**
- `/mavros/` prefix = all MAVROS topics
- `global_position/local` = estimated position in local frame (meters from starting point)
- `global_position/global` = GPS coordinates (latitude, longitude, altitude)
- `setpoint_*` = topics where you send commands to control the drone

| Topic | Message Type | Description | Use Case |
|-------|-------------|-------------|----------|
| `/mavros/state` | `mavros_msgs/State` | Connection status, mode, armed state | Check if drone is ready |
| `/mavros/global_position/local` | `nav_msgs/Odometry` | Local position estimate (NED frame) | Get drone position for navigation |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | GPS coordinates | Get lat/lon for outdoor missions |
| `/mavros/imu/data` | `sensor_msgs/Imu` | IMU data (orientation, angular velocity, linear acceleration) | Get attitude for stabilization |
| `/mavros/battery` | `sensor_msgs/BatteryState` | Battery voltage and percentage | Monitor power levels |
| `/mavros/altitude` | `mavros_msgs/Altitude` | Altitude estimates | Height above ground/sea level |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Send position commands | "Go to X,Y,Z coordinates" |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | Send velocity commands | "Move at X m/s forward" |

**Important Distinction:**
- **Subscriber topics** (you read from): `/mavros/global_position/*`, `/mavros/state`, `/mavros/battery`
- **Publisher topics** (you write to): `/mavros/setpoint_*` - these control the drone

### Testing the Full Stack

#### In Terminal 2 (MAVProxy/SITL):
```bash
mode guided
arm throttle
takeoff 5
```

#### In Terminal 3 (Monitor with ROS):
```bash
# Open new terminal tab: Ctrl+Shift+T
rostopic echo /mavros/global_position/local
```

You should see position values changing as the drone takes off!

---

## Understanding Coordinate Frames

### NED vs ENU Frames

When working with drones, understanding coordinate frames is crucial:

**NED (North-East-Down)** - Used by ArduPilot
- X points North
- Y points East
- Z points Down (into the ground)
- Altitude is negative (flying at 10m = Z = -10)

**ENU (East-North-Up)** - Used by ROS
- X points East
- Y points North
- Z points Up (into the sky)
- Altitude is positive (flying at 10m = Z = 10)

**Why This Matters:**
MAVROS automatically converts between NED (ArduPilot) and ENU (ROS), so when you read `/mavros/global_position/local`, you get positions in ENU (ROS standard). This is transparent to you, but important to understand if debugging position issues.

---