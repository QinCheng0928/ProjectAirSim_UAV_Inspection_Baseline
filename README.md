# ProjectAirSim UAV Inspection Baseline

<h4 align="center">
    <p>
        <b>English</b> | 
        <a href="https://github.com/QinCheng0928/ProjectAirSim_UAV_Inspection_Baseline/blob/main/docs/README.md">简体中文</a>
    </p>
</h4>

This project provides a baseline system for autonomous inspection of drones based on random strategies, capable of performing simple navigation, obstacle avoidance, and image acquisition in urban road network environments. The system employs a depth camera for obstacle detection and implements random path planning based on the topological structure of the road network.

##  Structure

```
ProjectAirSim_UAV_Inspection_Baseline/
├── main.py                          # Program main entrance
├── inspection.py                    # Inspection Manager Main Logic
├── config/
│   └── settings.py                  # Project configuration parameters
├── models/
│   ├── intersection.py              # Road Intersection Data Model
│   └── roadnetwork.py               # Road Network Topology Management
├── components/
│   ├── collision_handler.py         # Collision Detection Processor
│   └── image_handler.py             # Image Processing and Display Management
├── docs/                            # Project Documents
└── media/                           # Video file output directory
```

## Installation requirements

- **Ubuntu 20.04**
- **Python 3.8**
- **Unreal Engine 5 (5.2)**
- **City Sample**
- **ProjectAirSim (v0.1.1)**
