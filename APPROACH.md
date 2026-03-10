# Approaching a macOS Native Swift Clone of MoveIt2

> **Reference project:** https://github.com/moveit/moveit2  
> **What MoveIt2 is:** An open-source ROS 2 framework for robot motion planning, manipulation, kinematics, collision checking, 3D perception, and hardware control. Used on 150+ robots by researchers and industry alike.

---

## 1. First — Scope Definition

MoveIt2 is not a single application; it is a large framework of ~30 ROS 2 packages written primarily in C++ (~86%) with Python bindings, tightly coupled to the ROS 2 middleware (DDS, lifecycle nodes, tf2, URDF toolchain, etc.). A full 1:1 clone is a multi-year effort for a large team.

Before writing a line of code I would force a clear decision on **what "clone" means** across three possible interpretations:

| Interpretation | Scope | Realistic Timeline |
|---|---|---|
| **A** — Standalone macOS planning tool (no ROS, wraps upstream C++ libs) | Core planning + viz, minus ROS middleware | 12–18 months solo |
| **B** — Native macOS GUI that bridges a local ROS 2 / MoveIt2 install | Mostly UI/UX work, ROS does the heavy lifting | 3–6 months |
| **C** — Ground-up Swift reimplementation of the full stack | Everything below, from scratch in Swift | 3–5+ years |

**Recommended starting point: Interpretation A.** Build a self-contained macOS app that bundles or dynamically links the proven C++ planning libraries (OMPL, FCL, KDL) and provides a native SwiftUI/Metal front-end — no ROS installation required.

---

## 2. High-Level Architecture

```
┌───────────────────────────────────────────────────────────────┐
│                    SwiftUI Application Layer                   │
│  (Setup Assistant · Planning Scene Editor · Visualizer · Logs)│
└──────────────────────────┬────────────────────────────────────┘
                           │ Swift ↔ C++ Interop (Swift 5.9+)
┌──────────────────────────▼────────────────────────────────────┐
│              Swift Planning Orchestration Layer                │
│  (MoveGroupInterface equivalent · Task Constructor · Servo)   │
└─────┬──────────┬──────────┬──────────┬────────────┬───────────┘
      │          │          │          │            │
  ┌───▼───┐ ┌───▼───┐ ┌────▼────┐ ┌───▼────┐ ┌────▼─────┐
  │ URDF/ │ │  IK   │ │ Motion  │ │Collision│ │ Scene /  │
  │ SRDF  │ │Plugin │ │Planner  │ │Checking │ │ Octomap  │
  │Parser │ │(KDL)  │ │(OMPL /  │ │  (FCL)  │ │ Monitor  │
  │       │ │       │ │CHOMP/   │ │         │ │          │
  └───────┘ └───────┘ │ Pilz)   │ └─────────┘ └──────────┘
                      └─────────┘
┌───────────────────────────────────────────────────────────────┐
│                   Metal / SceneKit / RealityKit                │
│              (3-D robot + environment visualization)           │
└───────────────────────────────────────────────────────────────┘
┌───────────────────────────────────────────────────────────────┐
│          Optional: Hardware / Simulator Bridge                 │
│    (ros2_control plugin, Gazebo socket, custom serial bus)     │
└───────────────────────────────────────────────────────────────┘
```

---

## 3. Core Subsystems & Proposed Implementation Strategy

### 3.1 Robot Model — URDF / SRDF Parsing
**MoveIt2 equivalent:** `moveit_core/robot_model`, `srdfdom`

- URDF is XML. Parse with `XMLParser` (Foundation) or a C++ wrapper around `urdfdom`.
- Build an in-memory kinematic tree of links and joints, mirroring `RobotModel` / `RobotState`.
- SRDF (Semantic Robot Description Format) adds planning groups, end-effectors, virtual joints, and pre-defined poses — parse alongside URDF.
- Store as a value-type `struct` hierarchy in Swift, with `Sendable` conformance for concurrency safety.

**Key question:** Do we write the URDF parser ground-up in Swift, or C-bridge `urdfdom`?

---

### 3.2 Kinematics
**MoveIt2 equivalent:** `moveit_kinematics`, KDL plugin, `RobotState::setFromIK`

- **Forward kinematics (FK):** Pure Swift — recursive DH-parameter or product-of-exponentials traversal. FK is deterministic and fast; no C++ bridge needed.
- **Inverse kinematics (IK):** Use Swift C++ interop to call [KDL (Kinematics and Dynamics Library)](https://github.com/orocos/orocos_kinematics_dynamics) directly, or implement a Newton-Raphson Jacobian pseudoinverse solver in Swift using Accelerate.framework (`LAPACK` / `BLAS`).
- Support for analytical IK for common robot geometries (6-DOF, PUMA-style) as a fast-path.
- Design a plugin protocol (`IKSolver`) so solvers are swappable.

---

### 3.3 Collision Checking
**MoveIt2 equivalent:** FCL (Flexible Collision Library), `CollisionWorld`, `AllowedCollisionMatrix`

- Link against [FCL](https://github.com/flexible-collision-library/fcl) via Swift Package Manager binary target or CMake-built xcframework.
- Expose an `@_silgen_name`-bridged Swift API: `checkCollision(state:request:result:) -> Bool`.
- Implement the **Allowed Collision Matrix (ACM)** as a `[LinkPair: CollisionPolicy]` dictionary in Swift.
- Support mesh (`.stl`, `.dae`), primitive shapes (box, sphere, cylinder), and Octomap point cloud objects.
- Collision checking is the most expensive operation (~90% of planning time); prioritize an async/actor-isolated design with Swift Concurrency.

---

### 3.4 Planning Scene
**MoveIt2 equivalent:** `moveit_core/planning_scene`, `PlanningSceneMonitor`

- A Swift `actor PlanningScene` that holds the robot state, world geometry, and ACM.
- Operations: add/remove collision objects, attach/detach objects to robot links, apply sensor deltas.
- Thread-safe diffs (like MoveIt's `applyPlanningSceneWorld`) modeled as Swift value-type snapshots.
- Optional: live updates from a depth camera feed (via AVFoundation + ARKit point clouds on macOS with compatible hardware).

---

### 3.5 Motion Planning
**MoveIt2 equivalent:** `moveit_planners` (OMPL, CHOMP, Pilz), `PlanningPipeline`

This is the algorithmic heart of MoveIt.

- **OMPL integration:** [OMPL](https://ompl.kavrakilab.org/) builds on macOS. Link via xcframework. OMPL has no concept of a robot; provide it with a Swift-backed `StateValidityChecker` that calls our collision pipeline.
- **Pilz Industrial Motion Planner:** Relatively simple — LIN (linear Cartesian), PTP (joint-space point-to-point), CIRC (circular arc) segments. Reimplement in pure Swift; the math is well-documented.
- **CHOMP:** Gradient-based trajectory optimizer. Can be implemented in Swift using Accelerate for matrix ops, or linked from upstream C++ source.
- **Planning Pipeline:** A Swift `PlanningPipeline` struct that chains request adapters (start-state bounds check, workspace bounds, time parameterization) around a planner.
- **Time Parameterization:** TOTG (Time-Optimal Trajectory Generation) and Ruckig algorithms — both have standalone C++ implementations suitable for xcframework wrapping.

---

### 3.6 MoveIt Servo (Real-Time Control)
**MoveIt2 equivalent:** `moveit_servo`

- Converts Cartesian velocity commands (twist) to joint velocity commands in real time.
- Requires a tight control loop (~1 kHz); use a dedicated Swift `Task` with `Clock` or a POSIX real-time thread via `pthread` bridging.
- Singularity detection, joint limit damping, and collision checking at servo rate.

---

### 3.7 MoveIt Task Constructor
**MoveIt2 equivalent:** `moveit_task_constructor`

- A composable task graph: stages of type `Generator`, `Propagator` (forward/backward), `Connector`, and `Container`.
- Model in Swift as a recursive enum or protocol hierarchy with `AsyncSequence`-based solution streaming.
- Interactive solution introspection UI is a natural fit for SwiftUI's declarative list/outline views.

---

### 3.8 3-D Visualization
**MoveIt2 equivalent:** RViz MoveIt plugin (`moveit_ros_visualization`)

This is where Swift/macOS shines versus the ROS ecosystem.

- **SceneKit** for general 3D robot rendering (URDF mesh loading via ModelIO `.stl` / `.dae` → `SCNNode`).
- **Metal** shaders for custom visualizations (trajectory ribbons, occupancy voxels, heatmaps).
- **RealityKit** for potential AR preview on Macs with Apple Silicon + connected devices.
- Interactive 6-DOF goal pose dragging with a custom `NSViewRepresentable` + `SCNView` gesture recognizer.
- Joint-space slider panel embedded in SwiftUI alongside the 3D view.

---

### 3.9 Setup Assistant
**MoveIt2 equivalent:** `moveit_setup_assistant`

A step-by-step configuration wizard for a new robot:

1. Load URDF
2. Generate self-collision matrix
3. Define virtual joints, planning groups, end-effectors
4. Define named poses
5. Configure ROS 2 controllers (or native hardware bridges)
6. Generate SRDF + config files

This maps naturally to a SwiftUI multi-step `NavigationStack` flow with a live 3D preview.

---

### 3.10 Warehouse / Trajectory Cache
**MoveIt2 equivalent:** `moveit_ros_warehouse`, `moveit_ros_trajectory_cache`

- SQLite via `warehouse_ros_sqlite` (can be used directly) or reimplemented using Swift's `SQLite3` C bridge.
- Store/recall planning scenes, robot states, and trajectories keyed by robot + scene hash.

---

## 4. Key Technology Decisions to Resolve

| Decision | Options | Trade-offs |
|---|---|---|
| C++ interop strategy | Direct Swift ↔ C++ (Swift 5.9+) vs. C Bridge header vs. ObjC++ wrapper | Swift C++ interop is cleaner but still maturing; C bridge is proven |
| OMPL delivery | Build from source as xcframework vs. Swift Package binary target vs. reimplement subset | xcframework most reliable; reimplementation for simple planners only |
| 3D engine | SceneKit vs. Metal (custom) vs. RealityKit | SceneKit fastest to prototype; Metal needed for performance at scale |
| IK solver | KDL (C++ bridge) vs. pure Swift Jacobian vs. Core ML learned IK | KDL bridge most accurate; ML-based IK interesting for over-actuated arms |
| Concurrency model | Swift actors for planning scene isolation + structured concurrency for planning tasks | Natural fit; planning is inherently async |
| ROS 2 bridge | None (standalone) vs. optional rclswift / DDS socket layer | Scope choice; adds huge complexity, but enables interop with real robots |
| Data formats | Import-only URDF/SRDF vs. native format + conversion | Import-only is pragmatic; native format is a long-term investment |

---

## 5. Proposed Build Phases

### Phase 1 — Core Robot Model + Visualizer (Foundation)
- [ ] Swift Package with URDF XML parser
- [ ] KinematicTree data model (links, joints, DH params)
- [ ] FK implementation + unit tests
- [ ] SceneKit-based URDF renderer (meshes + joint articulation)
- [ ] SwiftUI shell app with 3D view + joint sliders

### Phase 2 — Collision + Planning Scene
- [ ] FCL xcframework integration
- [ ] CollisionWorld actor
- [ ] PlanningScene with add/remove world objects
- [ ] ACM implementation
- [ ] Self-collision matrix generator (for Setup Assistant)

### Phase 3 — Kinematics + IK
- [ ] KDL bridge or pure Swift Jacobian IK
- [ ] IKSolver protocol + plugin registry
- [ ] End-effector pose drag in 3D view → IK → joint update

### Phase 4 — Motion Planning
- [ ] OMPL xcframework build + StateValidityChecker bridge
- [ ] PlanningPipeline with adapters
- [ ] Pilz LIN / PTP reimplementation
- [ ] Time parameterization (TOTG or Ruckig)
- [ ] Trajectory playback in visualizer

### Phase 5 — Setup Assistant + Task Constructor
- [ ] SwiftUI Setup Assistant wizard
- [ ] Task Constructor stage model + solution tree UI

### Phase 6 — Hardware Bridge (Optional)
- [ ] ros2_control socket adapter
- [ ] Custom serial/CAN bus protocol plugin
- [ ] MoveIt Servo real-time loop

---

## 6. Open Questions

These must be answered before or early in development:

### Scope & Intent
1. **Is the goal a developer tool, a research tool, or an end-user product?** This changes UI investment priority significantly.
End-user product
2. **Does the app need to control real hardware, or is simulation + planning sufficient as v1?**
Simulation + planning is sufficient for v1
3. **Must it be ROS-interoperable (can connect to a running ROS 2 system), or fully standalone?**
Fully standalone
4. **Is MoveIt Pro (the commercial PickNik offering) a competitive concern?** If so, what differentiates this?
Free, ease of installation and use for beginner robotics student.

### Technical Architecture
5. **What is the minimum robot type to support first?** (6-DOF serial arm? Mobile manipulator? Parallel mechanisms?) Starting narrow lets us ship faster.
Pick a single 6-DOF serial arm that is OSS and widely understood.
6. **URDF vs. other formats:** Must we support only URDF, or also USD (Universal Scene Description, native to Apple's ecosystem), MuJoCo MJCF, or others?
Just URDF for V1
7. **How is sensor data fed in?** (ARKit, connected USB depth camera, synthetic from simulation?) This shapes the Perception subsystem entirely.
Synthetic from simulation. V1 planning scene obstacles are placed manually by the user via a UI panel — drag-to-place box/cylinder/sphere primitives. No live sensor feed in V1.
8. **Is there a target robot or use case to validate against?** (e.g., UR5, Franka, Unitree arm) A concrete test robot drives every design decision.
Pick one that is well understood.

### Libraries & Licensing
9. **OMPL is BSD-licensed — acceptable. FCL is BSD — acceptable. KDL is LGPL-2.1** — is dynamic linking acceptable, or does the license need to be cleared with stakeholders?
Dynamic linking is accepptable
10. **Will we ship with pre-built xcframeworks for OMPL/FCL/KDL, or require users to build C++ dependencies?** (The latter kills casual adoption on macOS.)
`build_deps.sh` script — runs once, compiles and packages OMPL/FCL/KDL xcframeworks into a Git-ignored `./deps/` directory. Requires Homebrew + CMake on first setup. No binary bloat in the repo.
11. **Can we use Apple's `vForce` / `BLAS` in Accelerate as a drop-in for Eigen** (used heavily in MoveIt's C++ core), or is a full Eigen port needed?
Yes

### Platform & Distribution
12. **macOS deployment target?** (Sonoma / Sequoia minimum? Affects Metal API availability, Swift Concurrency features.)
macOS Tahoe 26.3.1 — already shipping and running on the development machine. Swift Concurrency not required for V1 but available.
13. **Apple Silicon only, or Universal Binary (arm64 + x86_64)?** Affects C++ dependency build matrix.
Apple Silicon only
14. **App Store distribution or direct download?** App Store sandboxing conflicts with loading robot meshes from arbitrary file paths.
Just local to this machine for V1
15. **Is an iOS/iPadOS companion app (e.g., AR visualization of robot workspace) in scope?**
No

### Data & Persistence
16. **Where are robot configurations stored?** User-local directory, CloudKit sync, or a Git-backed bundle?
Local directory
17. **Is multi-robot scenario support needed** (more than one robot in the same scene)?
Not for V1

### Testing & Validation
18. **How do we validate planner correctness?** (Compare trajectory outputs against upstream MoveIt2 for the same URDF + start/goal configs?)
You decide
19. **Is there a benchmark dataset or test robot suite we must pass before calling a feature "done"?**
Please suggest how to resolve

---

## 7. Biggest Technical Risks

| Risk | Likelihood | Mitigation |
|---|---|---|
| Swift ↔ C++ interop immaturity for complex template-heavy libraries (FCL, OMPL) | High | Use C bridge headers, hide templates behind plain-C API |
| OMPL / FCL / KDL cross-compilation for macOS arm64 | Medium | Test CI on macOS from day 1; use CMake ExternalProject + fat xcframework |
| Real-time control loop timing on macOS (MoveIt Servo equivalent) | High | Use POSIX `pthread` with real-time scheduling; avoid GCD for the hot path |
| Rendering performance with large robot meshes + dense point clouds | Medium | Adopt Metal instanced rendering early; SceneKit does not scale to millions of voxels |
| Keeping pace with URDF/SRDF format evolution and community robot models | Low-Medium | Test against robot_description packages (UR, Franka, Spot) continuously |
| Scope creep (ROS bridge, iOS app, ML-IK, …) | High | Ruthlessly phase-gate features; ship Phase 1–2 as a usable product first |

---

## 8. All Decisions Resolved — Build Kickoff Order

All scoping and architecture questions are answered. The project is ready to start. Execute in this order:

### Step 1 — Repo & Dependency Scaffolding
- [ ] Create Xcode project: `MoveItMac`, SwiftUI + SceneKit, target macOS Tahoe (26.3.1), Apple Silicon
- [ ] Write `build_deps.sh` to clone, CMake-configure, and build OMPL (BSD), FCL (BSD), and KDL (LGPL-2.1, dynamically linked) as arm64 xcframeworks into `./deps/` (Git-ignored)
- [ ] Add `deps/` to `.gitignore`; add a `README` section for first-time setup (`bash build_deps.sh`)
- [ ] Verify all three xcframeworks link cleanly into a Hello World Swift target

### Step 2 — URDF Parser (Swift Package)
- [ ] New Swift Package `URDFKit` inside the repo
- [ ] Parse URDF XML into a `RobotModel` struct tree (links, joints, geometry, limits)
- [ ] Parse SRDF (planning groups, end-effectors, named states, ACM disable pairs)
- [ ] Unit-test against the **UR5e** URDF from `ros-industrial/universal_robot` (BSD)
- [ ] Unit-test against one other well-known robot (e.g., Franka Panda) to catch parser edge cases

### Step 3 — SceneKit Visualizer + FK
- [ ] Load UR5e URDF meshes (`.stl` → ModelIO → `SCNNode`) in a `RobotSceneView`
- [ ] Implement FK in pure Swift (product-of-exponentials); drive `SCNNode` transforms
- [ ] SwiftUI joint-slider panel (one slider per joint, live 3D update)
- [ ] Orbit/pan/zoom camera via `SCNCameraController`

### Step 4 — Planning Scene Editor
- [ ] SwiftUI inspector panel: add box / cylinder / sphere with position + dimensions
- [ ] Render obstacles as `SCNNode` in the same SceneKit scene
- [ ] Wire obstacles into the FCL `CollisionWorld` so they participate in collision checks

### Step 5 — Collision Checking
- [ ] C-bridge header over FCL; expose `checkCollision(robotState:world:) -> Bool`
- [ ] `CollisionWorld` Swift actor wrapping FCL
- [ ] ACM as `[LinkPair: CollisionPolicy]`; pre-generate self-collision matrix for UR5e
- [ ] Visual feedback: highlight colliding links red in the 3D view

### Step 6 — IK + End-Effector Drag
- [ ] KDL xcframework bridge (dynamic link, LGPL-2.1 compliant)
- [ ] `IKSolver` protocol; default impl delegates to KDL
- [ ] 6-DOF end-effector drag handle in SceneKit → IK → joint update → FK re-render

### Step 7 — Motion Planning
- [ ] OMPL xcframework + Swift `StateValidityChecker` callback into FCL collision pipeline
- [ ] `PlanningPipeline` Swift struct with pre/post adapters (bounds check, time parameterization)
- [ ] Pilz PTP + LIN planners in pure Swift (Accelerate for matrix ops)
- [ ] Ruckig time parameterization (C++ xcframework or Swift port)
- [ ] Trajectory playback animation in the 3D view

### Step 8 — Setup Assistant
- [ ] SwiftUI `NavigationStack` wizard: Load URDF → Generate ACM → Define Groups → Named Poses → Export SRDF
- [ ] Live 3D preview at every step

### Validation Gate (after Step 7)
Run the 10 UR5e acceptance tests before calling planning "done":

1. Home → pre-defined named target (no obstacles)
2. Joint-constrained reach (elbow-up constraint)
3. Near-singular wrist configuration
4. Reach with one box obstacle in the path
5. Reach with three obstacles (cluttered scene)
6. Start state in self-collision → planner must recover or reject gracefully
7. Goal state in collision with world object → planner must reject
8. Pilz PTP: joint-space point-to-point
9. Pilz LIN: straight-line Cartesian move
10. IK-dragged goal → plan → execute animation end-to-end

All 10 must succeed before Step 7 is closed.
