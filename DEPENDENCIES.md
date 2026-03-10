# Non-Swift Library Dependencies

All libraries below are Apple system frameworks (no third-party packages). URDFKit is
the one first-party Swift package in this repo and is excluded from this list.

---

## Libraries Inherited from MoveIt2

MoveIt2 is the upstream ROS 2 motion planning framework this project clones. The
libraries below are MoveIt2's own core dependencies. This project does not link them
directly — it reimplements or replaces each concept using the Apple frameworks listed
further below.

---

### Eigen3
**Language:** C++ header-only template library  
**MoveIt2 role:** The de-facto linear algebra backbone of MoveIt2. Used for every
dense matrix and vector operation: rotation matrices, homogeneous transforms, Jacobian
computation, pseudo-inverse (via `completeOrthogonalDecomposition`), and SVD for
damped least-squares IK.  
**This project's equivalent:** `simd` (hardware-accelerated SIMD math) + handwritten
4×4 matrix helpers in `Math.swift`.

---

### urdfdom / urdfdom_headers
**Language:** C++  
**MoveIt2 role:** The reference URDF parser used by ROS 2. Reads Unified Robot
Description Format XML into in-memory link/joint trees, resolves parent–child
relationships, and exposes joint limits, axis vectors, and origin transforms.  
**This project's equivalent:** `URDFKit` — the first-party Swift package that parses
URDF XML via `Foundation.XMLParser` and produces `RobotModel`, `Link`, and `Joint`
value types.

---

### srdfdom
**Language:** C++  
**MoveIt2 role:** Parses the Semantic Robot Description Format (SRDF) — a companion
file to the URDF that names planning groups, defines allowed collision pairs, and
stores named robot states (e.g. "home", "ready"). MoveIt2 relies on srdfdom to know
which joint subsets to plan for and which self-collisions to permanently ignore.  
**This project's equivalent:** Not yet implemented. The "Named states" feature on the
roadmap would cover this.

---

### KDL (Orocos Kinematics and Dynamics Library)
**Language:** C++  
**MoveIt2 role:** Provides the kinematic chain abstraction (`KDL::Chain`), forward
kinematics solver (`ChainFkSolverPos_recursive`), and the Jacobian solver
(`ChainJntToJacSolver`) that MoveIt2's default KDL IK plugin builds on.  
**This project's equivalent:** `ForwardKinematics.swift` (recursive parent-chain FK)
and the numerical Jacobian computed via finite differences in `IKSolver.swift`
(upcoming).

---

### OMPL (Open Motion Planning Library)
**Language:** C++  
**MoveIt2 role:** The sampling-based motion planning engine behind MoveIt2's planners.
Provides RRT, RRT*, PRM, EST, and dozens of other algorithms. MoveIt2 wraps OMPL
behind its `planning_interface` and adds collision-aware state validation.  
**This project's equivalent:** Not yet implemented. The RRT planner on the roadmap
will reimplement this concept from scratch in Swift.

---

### FCL (Flexible Collision Library)
**Language:** C++  
**MoveIt2 role:** MoveIt2's default collision detection backend. Handles narrow-phase
geometry queries (mesh–mesh, box–sphere, etc.) for both self-collision and
scene-object collision, supporting continuous collision detection and distance queries.  
**This project's equivalent:** `CollisionChecker.swift` — a bounding-sphere O(n²)
approximation sufficient for interactive use, replacing FCL's exact mesh queries.

---

### OctoMap
**Language:** C++  
**MoveIt2 role:** A probabilistic 3-D occupancy grid stored as an octree. MoveIt2 uses
it to represent the environment as sensed by depth cameras or LIDAR; the octree is
fed into the collision checker so the planner avoids real-world obstacles in the
point cloud, not just explicitly modeled ones.  
**This project's equivalent:** The `Obstacle` struct in `AppState` — a simple
explicit obstacle list. Probabilistic occupancy mapping is not planned.

---

### tf2 (Transform Library)
**Language:** C++ / Python (ROS 2 package)  
**MoveIt2 role:** Maintains a time-stamped tree of coordinate frame transforms across
the whole robot system. MoveIt2 uses tf2 to look up where any link or sensor is in
world space at any past or present timestamp, and to broadcast the current robot state
for other ROS nodes to consume.  
**This project's equivalent:** The SceneKit node hierarchy — each `SCNNode`'s
`simdWorldTransform` implicitly maintains the same parent-to-world chain as tf2, but
without timestamps or ROS message broadcasting.

---

---

## AppKit
**Module:** `AppKit`  
**Source:** Apple system framework (macOS only)

The foundational AppKit UI toolkit for macOS. Provides `NSView`, `NSSlider`,
`NSTextField`, `NSScrollView`, `NSOpenPanel`, `NSColor`, and the run-loop infrastructure
(`NSRunLoop`, `NSEventTrackingRunLoopMode`). Used directly for the joint slider panel
because SwiftUI's higher-level wrappers interfered with AppKit's mouse-tracking loop
during drag events.

---

## SceneKit
**Module:** `SceneKit`  
**Source:** Apple system framework (macOS / iOS / tvOS)

Apple's retained-mode 3-D scene graph framework built on Metal. Provides `SCNScene`,
`SCNNode`, `SCNGeometry`, `SCNView`, and the `SCNSceneRendererDelegate` protocol
(including `renderer(_:updateAtTime:)` which runs on the CVDisplayLink thread). Used
for the robot 3-D viewport — joint transforms, link geometry display, and collision
tinting.

---

## simd
**Module:** `simd`  
**Source:** Apple system library (part of the Accelerate umbrella)

A low-level SIMD math library providing hardware-accelerated vector and matrix types:
`simd_double4x4`, `simd_float4x4`, `simd_double3`, `simd_quatd`, etc. Used throughout
the forward kinematics engine and IK solver for transform composition, rotation
matrices, and homogeneous coordinate math.

---

## Foundation
**Module:** `Foundation`  
**Source:** Apple system framework (cross-platform)

Apple's base layer providing `URL`, `Timer`, `RunLoop`, `XMLParser`, `Date`,
`NotificationCenter`, `DispatchQueue`, and `NSLock`. Used for URDF file loading
(`XMLParser`), the 120 Hz `.common`-mode `Timer`, and the `NSLock` that guards
the joint-angle snapshot between the main thread and the CVDisplayLink render thread.

---

## SwiftUI
**Module:** `SwiftUI`  
**Source:** Apple system framework (macOS 10.15+)

Apple's declarative UI framework. Provides the three-column `NavigationSplitView`,
`@EnvironmentObject`, `@Published`/`ObservableObject`, and `NSViewRepresentable` (the
bridge that embeds AppKit views inside a SwiftUI hierarchy). The main app shell,
sidebar, and detail column are SwiftUI; the performance-critical slider rows and 3-D
viewport are bridged AppKit/SceneKit views to avoid SwiftUI re-render interference
during drag.
