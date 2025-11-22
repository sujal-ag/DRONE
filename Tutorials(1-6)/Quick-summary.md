## Complete System Architecture

Now that we've set everything up, here's how all pieces fit together:

```
┌─────────────────────────────────────────────────────────────┐
│                    Your Autonomy Code                        │
│              (Python/C++ ROS Nodes)                          │
│   - Path planning                                            │
│   - Object detection                                         │
│   - Decision making                                          │
└────────────┬────────────────────────────┬───────────────────┘
             │                            │
             │ Publish commands           │ Subscribe to telemetry
             │ (/mavros/setpoint_*)       │ (/mavros/state, position, etc.)
             │                            │
┌────────────▼────────────────────────────▼───────────────────┐
│                         ROS Master                           │
│              (Message routing system)                        │
└────────────┬─────────────────────────────────────────────────┘
             │
┌────────────▼────────────────────────────────────────────────┐
│                         MAVROS                               │
│        (Translates ROS ↔ MAVLink)                           │
└────────────┬────────────────────────────────────────────────┘
             │
             │ MAVLink messages
             │
┌────────────▼────────────────────────────────────────────────┐
│                    ArduPilot SITL                            │
│   (Flight control firmware - keeps drone stable)            │
│   - Sensor fusion                                            │
│   - PID control loops                                        │
│   - Safety features                                          │
└────────────┬────────────────────────────────────────────────┘
             │
             │ Motor commands & sensor requests
             │
┌────────────▼────────────────────────────────────────────────┐
│                    Gazebo Simulator                          │
│   - Physics engine (gravity, aerodynamics)                  │
│   - 3D visualization                                         │
│   - Sensor simulation (camera, LiDAR, GPS, IMU)            │
└──────────────────────────────────────────────────────────────┘
```

---

## Quick Reference: Complete Workflow

### Starting Everything (3 Terminals)

**Terminal 1 - Gazebo:**
```bash
roslaunch iq_sim runway.launch
```

**Terminal 2 - SITL:**
```bash
~/startsitl.sh
```

**Terminal 3 - MAVROS:**
```bash
roslaunch iq_sim apm.launch
```

### Common Troubleshooting Commands

```bash
# Check if ROS master is running
rosnode list

# Kill all ROS nodes
rosnode kill -a

# Check Gazebo processes
ps aux | grep gazebo

# Kill Gazebo if stuck
killall -9 gazebo gzserver gzclient

# Rebuild catkin workspace if packages not found
cd ~/catkin_ws
catkin clean
catkin build
source ~/.bashrc

# Check MAVROS connection
rostopic echo /mavros/state
```

---

## Next Steps

Now that you have the complete simulation stack running, you can:

1. **Write Python/C++ ROS nodes to control the drone** - Next tutorial!
2. **Implement autonomous navigation algorithms** - Waypoint following, path planning
3. **Integrate computer vision** - Object detection, line following, AprilTag landing
4. **Test obstacle avoidance** - Using LiDAR data for collision prevention
5. **Deploy to real hardware** - Pixhawk + Companion Computer (Raspberry Pi/Jetson)

### What We've Accomplished

✅ **ArduPilot SITL** - Can simulate flight controller firmware
✅ **Gazebo** - 3D physics simulation with sensors
✅ **ROS & Catkin** - Framework for writing autonomy code
✅ **MAVROS** - Bridge to communicate with ArduPilot
✅ **QGroundControl** - Visual interface for monitoring

**You can now:**
- Simulate drones without hardware
- Access all telemetry data
- Visualize flight in 3D
- Test code safely before flying real drones

---

## Common Issues and Solutions

### Issue: `sim_vehicle.py: command not found`
**Solution:**
```bash
# Make sure you sourced your profile after installation
source ~/.profile
# Or add ArduPilot to PATH manually
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
source ~/.bashrc
```

### Issue: Gazebo crashes or is very slow
**Solution:**
```bash
# Close all Gazebo processes
killall -9 gzserver gzclient

# Reduce graphics quality
export LIBGL_ALWAYS_SOFTWARE=1  # Software rendering

# Or use headless mode (no GUI, faster)
gzserver --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

### Issue: MAVROS shows "FCU: not connected"
**Solution:**
```bash
# Make sure SITL is running first, then start MAVROS
# Check if SITL is outputting MAVLink on port 14551
# In SITL terminal, you should see "Telemetry link 1 active"

# Check MAVROS connection params
rostopic echo /mavros/state
# connected: True means it's working
```

### Issue: `catkin build` fails with missing dependencies
**Solution:**
```bash
# Update rosdep database
rosdep update

# Install all dependencies
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# Clean and rebuild
catkin clean
catkin build
```

### Issue: `rostopic list` shows no topics
**Solution:**
```bash
# Check if ROS master is running
rosnode list

# If empty, launch a ROS launch file first
roslaunch iq_sim runway.launch

# Or start ROS master manually
roscore
```

---

## Glossary of Terms

| Term | Definition |
|------|------------|
| **SITL** | Software-In-The-Loop - simulates flight controller firmware on your PC |
| **Gazebo** | 3D robotics simulator with physics engine |
| **ROS** | Robot Operating System - middleware for robotics development |
| **Catkin** | Build system for ROS packages |
| **MAVLink** | Lightweight communication protocol for drones |
| **MAVROS** | ROS package that translates MAVLink to ROS messages |
| **Flight Controller** | Hardware that stabilizes the drone (Pixhawk, etc.) |
| **Companion Computer** | Linux computer for AI and autonomy (Jetson, Raspberry Pi) |
| **GCS** | Ground Control Station - software to monitor and control drone |
| **Topic** | Named channel for message passing in ROS |
| **Node** | A running ROS program (can be Python or C++) |
| **Publisher** | ROS node that sends messages on a topic |
| **Subscriber** | ROS node that receives messages from a topic |
| **NED Frame** | North-East-Down coordinate system (ArduPilot uses this) |
| **ENU Frame** | East-North-Up coordinate system (ROS uses this) |
| **Odometry** | Position and velocity estimate from sensors |

---

## Additional Resources

- **ArduPilot Documentation:** https://ardupilot.org/copter/
- **MAVROS Documentation:** http://wiki.ros.org/mavros
- **Gazebo Tutorials:** http://gazebosim.org/tutorials
- **ROS Tutorials:** http://wiki.ros.org/ROS/Tutorials
- **IQ Simulation GitHub:** https://github.com/Intelligent-Quads/iq_sim