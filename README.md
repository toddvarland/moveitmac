# MoveIt Mac

A native macOS motion planning tool — a Swift/SwiftUI reimagination of [MoveIt2](https://github.com/moveit/moveit2), built for robotics students and researchers. No ROS installation required.

> **Status:** Phase 1 scaffold — URDF parser + 3-D visualizer shell in progress.  
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
| 1 | 🔨 In progress | URDF parser + SceneKit visualizer shell |
| 2 | ⬜ Not started | Collision checking (FCL integration) |
| 3 | ⬜ Not started | IK + end-effector drag (KDL) |
| 4 | ⬜ Not started | Motion planning (OMPL + Pilz) |
| 5 | ⬜ Not started | Setup Assistant wizard |
| 6 | ⬜ Not started | Hardware bridge (optional) |

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
