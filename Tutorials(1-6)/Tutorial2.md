## Tutorial 2: Installing ArduPilot and Running SITL

### What is SITL?
**Software-In-The-Loop (SITL)** is a virtual environment that simulates drone flight and physics, allowing you to test missions and code safely without hardware.

**Why SITL is Essential:**
- ✅ Test code without risking expensive hardware
- ✅ Develop anywhere, even without a drone
- ✅ Iterate quickly - crash in simulation, not in real life
- ✅ Debug algorithms in a controlled environment
- ✅ Simulate GPS denied environments, sensor failures, wind conditions

**How it Works:**
SITL runs the exact same ArduPilot firmware that runs on real flight controllers, but instead of talking to real sensors and motors, it talks to a physics simulator. This means your code tested in SITL will work identically on real hardware.

### Installation Steps

#### Step 1: Open Terminal
```bash
# Press Ctrl + Alt + T to open terminal
```

#### Step 2: Install Git (if not already installed)
```bash
sudo apt update
sudo apt install git -y
```

#### Step 3: Clone ArduPilot Repository
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

#### Step 4: Install Dependencies
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

#### Step 5: Reload Your Profile
```bash
. ~/.profile
```

#### Step 6: (Optional) Fix Submodule Issues
If you encounter issues with `git submodule update`, run:
```bash
git config --global url.https://.insteadOf git://
```

#### Step 7: Checkout Stable Copter Version
```bash
git checkout Copter-4.0.4
git submodule update --init --recursive
```

#### Step 8: Run SITL for the First Time
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```
This initializes parameters. Wait for it to complete, then close with `Ctrl+C`.

#### Step 9: Run SITL Normally
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py --console --map
```

### Basic SITL Commands (MAVProxy Console)

Once SITL is running, try these commands:

```bash
# Switch to guided mode
mode guided

# Arm the drone
arm throttle

# Take off to 10 meters (you need to be quick otherwise the drone will be disarmed)
takeoff 10

# Check current position
position

# Return to launch point
mode rtl

# Land
mode land

# Disarm (only when landed)
disarm
```

---