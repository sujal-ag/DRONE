## Tutorial 3: Installing and Using QGroundControl

### What is QGroundControl?
A GUI-based ground station that provides visual mission planning and real-time telemetry monitoring.

**Why Use QGroundControl?**
While MAVProxy (the terminal-based interface) is powerful, QGroundControl provides:
- **Visual mission planning** - Click on map to create waypoints
- **Real-time telemetry display** - See altitude, speed, battery at a glance
- **Intuitive controls** - Arm, takeoff, land with buttons
- **Tutorial feed integration** - View camera streams
- **Parameter tuning** - Adjust PID gains and settings easily

**QGroundControl vs Mission Planner:**
- QGroundControl: Cross-platform (Linux, Windows, Mac), modern UI
- Mission Planner: Windows-only, more advanced features but steeper learning curve
- We use QGroundControl for its Linux compatibility and ease of use

### Installation Steps

#### Step 1: Download QGroundControl
```bash
cd ~
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
```

#### Step 2: Make it Executable
```bash
chmod +x QGroundControl.AppImage
```

#### Step 3: Run QGroundControl
```bash
./QGroundControl.AppImage
```

### Connecting QGroundControl to SITL

#### Terminal 1: Launch SITL
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py --console
```

#### Terminal 2: Launch QGroundControl
```bash
./QGroundControl.AppImage
```

QGroundControl will automatically connect to the SITL drone on UDP port 14550.

### Creating Your First Mission

1. Click **"Plan"** tab in QGroundControl
2. Click on the map to add waypoints:
   - **Waypoint 1:** Takeoff (set altitude: 50m)
   - **Waypoint 2:** Navigation point (5m away)
   - **Waypoint 3:** Land
3. Click **"Upload"** to send mission to drone
4. Switch to **"Fly"** view
5. Arm the drone and switch to **AUTO** mode
6. Click **"Start Mission"**

---