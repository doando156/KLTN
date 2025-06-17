# Multi-Robot Leader-Follower Simulation & Control (MATLAB GUI)

> **Team 3 – Graduation Thesis**  
> Experimental platform for multi differential-drive robots, controlled via MATLAB, testing the leader-follower algorithm.

---

## 🚀 Project Overview
This project provides a **MATLAB GUI** that enables:
- **Simulation** and **real-world control** of a team of differential-drive robots under a **leader-follower** formation.
- **Adding/removing** robots and waypoints; **loading scenarios** from JSON files; operation in two modes: **Simulation** and **Real-world**.
- **Monitoring**, **visualization**, and **export** of data: positions, trajectories, elapsed time, and distances traveled.

---

## 📂 Folder Structure

```
/
├─ Code/  
│   ├─ main.m            # Main GUI, callbacks, and control logic  
│   ├─ main.fig          # GUIDE layout file  
│   ├─ draw_robot_trajectory.m  
│   ├─ Draw_Path_LF.m    
│   ├─ Draw_path.m       
│   └─ analyst_comunication.m  
│
├─ paths/                # Stores CSV trajectory data for each experiment  
│   ├─ experiment1/  
│   ├─ experiment2/  
│   ├─ experiment3/  
│   └─ experiment4/  
│
├─ Position_tracking/    # JSON files for robot positions  
│   ├─ current_position.json            # Simulation mode  
│   └─ current_position_robot_simu.json # Updated during simulation  
│
├─ Scenario/             # Experiment scenarios (JSON)  
│   ├─ scenario1.json  
│   ├─ test_scenario1.json  
│   └─ experiment*.json  
│
├─ Target/               # Waypoints (JSON)  
│   └─ target_position.json  
│
├─ resources/project/    # Project configuration files (Project.xml, etc.)  
│   ├─ Project.xml  
│   ├─ rootp.xml  
│   └─ uuid-*.xml  
│
├─ .gitignore  
├─ README.md  

```

---

## 🔧 Requirements & Installation

1. **MATLAB R2018a** (or later) with GUIDE and basic toolboxes.
2. **USB/Serial** or **Bluetooth** connection for accessing physical robots.
3. Clone the repository:
   ```bash
   git clone https://github.com/USERNAME/multi-robot-matlab-gui.git
   cd multi-robot-matlab-gui
   ```
4. Open `main.m` in MATLAB and click **Run**, or double-click `main.fig` to launch the GUI.

---

## ⚙️ Usage Guide

1. Select **mode** (Simulation / Real-world) from the dropdown.
2. Click **Init**: Initialize the workspace, load robot positions and waypoints.
3. Click **Start**: Begin simulation or real-world control.
4. **Stop/Continue**: Pause or resume the run.
5. **Show Path**: Toggle trajectory visualization.
6. **Add Robot / Add Target**: Manually add robots or waypoints.
7. **Select Scenario**: Load a scenario from the `Scenario/` folder.
8. **Save Scenario**: Export the current setup to a JSON file.
9. **Export Path**: Export trajectories and summary CSV files into `paths/`.

---

## 📊 Data Formats

- **Robot data JSON** (`current_position.json` / camera):
  ```json
  [
    { "id": 1, "x": 0.5, "y": 1.2, "theta": 45 },
    { "id": 2, "x": 1.0, "y": 0.2, "theta": 90 }
  ]
  ```
- **Waypoint data JSON** (`target_position.json`):
  ```json
  [
    { "x": 2.0, "y": 3.0 },
    { "x": 4.5, "y": 1.0 }
  ]
  ```

---

## 🤝 Contributing
1. Fork this repository.  
2. Create a feature branch:
   ```bash
   git checkout -b feature/your-feature
   ```
3. Commit and push:
   ```bash
   git commit -am "Add feature description"
   git push origin feature/your-feature
   ```
4. Open a Pull Request and describe your changes.

---

## 👨‍💻 Authors
- Doan Do

## 📝 License
The project is used for learning - research.