# Autonomous Drone Development - Complete Guide

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-4.0.4-red.svg)](https://ardupilot.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-9%20%7C%2011-orange.svg)](http://gazebosim.org/)

> **A comprehensive, beginner-friendly tutorial series that takes you from zero to building autonomous drones.** Learn to set up simulation environments, write control algorithms, and deploy to real hardware. Each tutorial builds on the previous one - follow in order for best results!

---

## What You'll Learn

- **Drone System Architecture** - Understand flight controllers, companion computers, and sensors
- **Simulation Setup** - Master ArduPilot SITL, Gazebo, and ROS integration
- **Autonomous Navigation** - Write Python/C++ code for waypoint missions and path planning
- **Sensor Integration** - Work with cameras, LiDAR, GPS, and IMU data
- **Real Hardware Deployment** - Transfer your code from simulation to physical drones

---

## Who This Is For

- **Beginners** - No prior drone or robotics experience required
- **Software Developers** - Learn robotics without hardware costs
- **Students & Researchers** - Academic-grade simulation environment
- **Hobbyists** - Build autonomous drones as a passion project
- **Hardware Engineers** - Understand the software side of drones

---

## Prerequisites

### Required
- Ubuntu 18.04 or 20.04 (native or VM recommended)
- Basic Linux terminal knowledge
- ~20GB free disk space
- 4GB+ RAM (8GB recommended for Gazebo)

### Helpful (but not required)
- Python or C++ programming basics
- Understanding of coordinate systems
- Git basics

---

## Quick Start

```bash
# Clone this repository
git clone https://github.com/sujal-ag/DRONE.git
cd DRONE

# Follow the tutorials in order
# Start with: Tutorials/Tutorial1.md
```

---

## Tutorial Series

### Core Setup Tutorials
1. **[Tutorial1.md](Tutorials(1-6)/Tutorial1.md)** - System Overview of Autonomous Drones
2. **[Tutorial2.md](Tutorials(1-6)/Tutorial2.md)** - Installing ArduPilot & Running SITL
3. **[Tutorial3.md](Tutorials(1-6)/Tutorial3.md)** - Installing and Using QGroundControl
4. **[Tutorial4.md](Tutorials(1-6)/Tutorial4.md)** - Installing Gazebo and ArduPilot Plugin
5. **[Tutorial5.md](Tutorials(1-6)/Tutorial5.md)** - Installing ROS & Catkin Workspace Setup
6. **[Tutorial6.md](Tutorials(1-6)/Tutorial6.md)** - ROS Basics, Topics, and Accessing Drone Telemetry

**[Tutorials-Summary.md](Tutorials/Tutorials-Summary.md)** - Quick reference guide for all tutorials

### Programming Tutorials (Coming Soon)
7. **Writing Your First Autonomous Script** - Python ROS nodes
8. **Waypoint Navigation** - Programmatic mission control
9. **Sensor Integration** - Camera and LiDAR processing
10. **Path Planning & Obstacle Avoidance** - Advanced autonomy

---

## 🛠️ Tech Stack

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Flight Controller Firmware** | ArduPilot 4.0.4 | Low-level flight control |
| **Simulator** | Gazebo 9/11 | 3D physics and sensor simulation |
| **Middleware** | ROS Noetic | Message passing and tools |
| **Communication** | MAVROS + MAVLink | Drone ↔ Computer interface |
| **Ground Station** | QGroundControl | Mission planning and monitoring |
| **Languages** | Python 3, C++ | Autonomy algorithms |

---

## Repository Structure

```
DRONE/
├── Tutorials/                     # Tutorial documentation
│   ├── Tutorial1.md              # System Overview
│   ├── Tutorial2.md              # ArduPilot & SITL
│   ├── Tutorial3.md              # QGroundControl
│   ├── Tutorial4.md              # Gazebo Setup
│   ├── Tutorial5.md              # ROS & Catkin
│   ├── Tutorial6.md              # ROS Basics & MAVROS
│   └── Tutorials-Summary.md      # Quick reference
├── scripts/                       # Helper scripts (coming soon)
├── examples/                      # Code examples (coming soon)
└── README.md                      # This file
```

---

## Video Tutorials

Each tutorial has an accompanying video walkthrough:

- **[Video Playlist](https://youtube.com/playlist/your-playlist-link)** - Complete series
- **Individual videos linked in each tutorial document**

**Subscribe for updates as new tutorials are released!**

---

## Installation

### Manual Setup (Recommended)
Follow the tutorials in order for detailed, step-by-step installation:
1. [ArduPilot Installation](Tutorials/Tutorial2.md)
2. [Gazebo Setup](Tutorials/Tutorial4.md)
3. [ROS & MAVROS Installation](Tutorials/Tutorial5.md)

> **Note:** Automated installation scripts coming soon!

---

## Testing Your Setup

After completing the setup tutorials, verify everything works:

```bash
# Terminal 1: Start Gazebo simulation
roslaunch iq_sim runway.launch

# Terminal 2: Start ArduPilot SITL
~/startsitl.sh

# Terminal 3: Start MAVROS
roslaunch iq_sim apm.launch

# Terminal 4: Check topics
rostopic list | grep mavros
```

You should see the drone in Gazebo and MAVROS topics listed!

---

## Contributing

Contributions are welcome! Here's how you can help:

- 🐛 **Report bugs** - Open an issue with reproduction steps
- 💡 **Suggest improvements** - Feature requests and tutorial ideas
- 📝 **Fix documentation** - Typos, clarifications, translations
- 🔧 **Submit code** - Examples, scripts, or bug fixes

### Contribution Guidelines
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## Troubleshooting

### Common Issues

**SITL won't start:**
```bash
source ~/.profile
cd ~/ardupilot/ArduCopter
sim_vehicle.py --console
```

**Gazebo crashes:**
```bash
killall -9 gzserver gzclient
# Restart with lower graphics
export LIBGL_ALWAYS_SOFTWARE=1
```

**MAVROS connection failed:**
```bash
# Check SITL is running first
rostopic echo /mavros/state
# Should show connected: True
```

**More solutions in:** [Tutorials/Tutorials-Summary.md](Tutorials/Tutorials-Summary.md)

---

## Additional Resources

### Official Documentation
- [ArduPilot Docs](https://ardupilot.org/copter/) - Flight controller firmware
- [ROS Wiki](http://wiki.ros.org/) - Robot Operating System
- [Gazebo Tutorials](http://gazebosim.org/tutorials) - Simulation platform
- [MAVROS Documentation](http://wiki.ros.org/mavros) - MAVLink bridge

### Community
- [ArduPilot Forum](https://discuss.ardupilot.org/) - Official support forum
- [ROS Discourse](https://discourse.ros.org/) - ROS community
- [Intelligent Quads](https://github.com/Intelligent-Quads) - iq_sim package

### Recommended Reading
- *Learning ROS for Robotics Programming* - Official ROS book
- *ArduPilot Development Guide* - Contribute to ArduPilot
- *Modern Robotics* - Theoretical foundations

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **ArduPilot Team** - For the robust open-source autopilot
- **ROS Community** - For the incredible robotics framework
- **Gazebo Team** - For the realistic simulator
- **Intelligent Quads** - For the iq_sim simulation package
- **All Contributors** - For improving these tutorials

---

## 📧 Contact & Support

- **Issues:** [GitHub Issues](https://github.com/sujal-ag/DRONE/issues)
- **Discussions:** [GitHub Discussions](https://github.com/sujal-ag/DRONE/discussions)

---

## ⭐ Star History

If you find this helpful, please ⭐ star this repository to help others discover it!

[![Star History Chart](https://api.star-history.com/svg?repos=sujal-ag/DRONE&type=Date)](https://star-history.com/#sujal-ag/DRONE&Date)

---

## 🗺️ Roadmap

- [x] Core setup tutorials (1-6)
- [ ] Python programming tutorials
- [ ] Advanced autonomy algorithms
- [ ] Computer vision integration
- [ ] Multi-drone swarms
- [ ] Real hardware deployment guide
- [ ] Video series completion
- [ ] Translations (Spanish, Chinese, etc.)

---

<p align="center">
  <b>Happy Flying! 🚁</b>
  <br>
  <i>Built with ❤️ by the drone community</i>
</p>
