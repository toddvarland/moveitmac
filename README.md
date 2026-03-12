# MoveIt Mac

A native macOS motion planning tool — a Swift/SwiftUI reimagination of [MoveIt2](https://github.com/moveit/moveit2), built for robotics students and researchers. No ROS installation required.

> **Status:** Active development — URDF parser, 3-D visualizer, collision checking, IK, RRT motion planning, and myCobot 280-M5 hardware bridge all working.  
> See [APPROACH.md](APPROACH.md) for the full architecture and build plan.

---

## Requirements

| Requirement | Version |
|---|---|
| macOS | Tahoe 26.0+ |
| Xcode | 26.3+ |
| CMake | 3.22+ (`brew install cmake`) |
| Git | any |
| xcodegen | 2.40+ (`brew install xcodegen`) |

---

## First-time Setup

```bash
# 1. Install build tools (if not already installed)
brew install cmake xcodegen

# 2. Build C++ xcframeworks and generate the Xcode project
make setup

# 3. Open in Xcode
open MoveItMac.xcodeproj
```

`make setup` runs `build_deps.sh` (≈ 10–20 min on first run) which:
- Clones and builds **OMPL** (BSD-3-Clause), **FCL** (BSD-3-Clause), and **KDL** (LGPL-2.1) as arm64 xcframeworks into `deps/`
- Regenerates `MoveItMac.xcodeproj` with the xcframework references activated

Subsequent runs skip already-built frameworks. To force a full C++ rebuild:
```bash
make clean-deps && make setup
```

---

## Running URDFKit Tests (no Xcode needed)

```bash
make test-urdf
```

This runs the `URDFKit` Swift package tests directly via `swift test`.

---

## Project Structure

```
moveit-clone/
├── MoveItMac/               SwiftUI application
│   ├── App/                 @main entry point
│   ├── Views/               ContentView, RobotSceneView, JointSliderPanel, SidebarView
│   ├── Model/               AppState, Obstacle
│   └── Resources/           Assets.xcassets
├── MoveItMacTests/          App-level unit tests
├── Packages/
│   └── URDFKit/             Pure-Swift URDF/SRDF parser (local Swift Package)
│       ├── Sources/URDFKit/ RobotModel, Link, Joint, URDFParser
│       └── Tests/           URDFParserTests (20 tests)
├── deps/                    Built xcframeworks — Git-ignored, created by build_deps.sh
├── .deps_build/             CMake intermediate artefacts — Git-ignored
├── project.yml              xcodegen project spec
├── Makefile                 Developer convenience targets
├── build_deps.sh            C++ dependency build script
└── APPROACH.md              Full architecture document and build plan
```

---

## Build Phases

| Phase | Status | Description |
|---|---|---|
| 1 | ✅ Done | URDF parser + SceneKit visualizer |
| 2 | ✅ Done | Collision checking (FCL / self-collision ACM) |
| 3 | ✅ Done | IK + forward kinematics (KDL) |
| 4 | ✅ Done | Motion planning (RRT + time parameterization) |
| 5 | ✅ Done | Setup Assistant wizard |
| 6 | ✅ Done | Hardware bridge (myCobot 280-M5 USB serial) |

---

## Testing the Full Pipeline

### 1. Basic Setup
- Launch the app
- In Setup Assistant, load `cobot.urdf` — robot name shows **myCobot280**
- Confirm the 3D arm renders in the scene view

### 2. Serial Connection
- Connect the myCobot 280-M5 via USB
- Select the port (e.g. `/dev/cu.usbserial-XXXXXXXX`) in the connection picker and connect
- Confirm the button toggles to **Disconnect**
- Press **Start** to activate the servo engine — jogging is not possible until Start is pressed

### 3. Physical → Virtual Mirroring
- With the arm connected and the servo engine started, jog a joint using the +/− buttons
- Release — confirm the virtual arm in the 3D view reflects the new position (~5 Hz poll)

### 4. Jogging
- In Servo Control Panel, hold a +/− jog button for a joint
- Confirm the physical arm moves and the UI angle value updates
- Release — arm should stop

### 5. Zero Pose
- Press **Zero Pose**
- Physical arm moves to 0° on all joints (home/calibration position); virtual arm follows

### 6. Trajectory Planning
- Press **Set Start** — captures current joint angles as the start pose
- Drag sliders or jog to a new goal position — virtual arm should **not** snap back
- Press **Set Goal**
- Press **Plan** — cyan EE path tube appears in the 3D view
- Scrub the trajectory preview slider to verify the motion

### 7. Execute on Robot
- After a successful plan, press **Execute on Robot**
- Physical arm moves through the planned trajectory at ~10 Hz
- Virtual arm tracks along simultaneously

### 8. Collision Avoidance
- Move sliders into a self-collision configuration
- Confirm the servo engine halts and shows a collision warning
- Return to a clear pose — jogging resumes

---

## License Compliance

| Library | License | Linking |
|---|---|---|
| OMPL | BSD-3-Clause | Static or dynamic |
| FCL | BSD-3-Clause | Static or dynamic |
| KDL | **LGPL-2.1** | **Dynamic only** — do not statically link |

KDL is dynamically linked as required by LGPL-2.1. The `build_deps.sh` script builds it as a `.dylib` and packages it as an xcframework with `embed: true`.

---

## Target Robot

**Universal Robots UR5e** — a 6-DOF serial arm used in the majority of MoveIt educational content. URDF available from the [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot) repository (BSD license).
