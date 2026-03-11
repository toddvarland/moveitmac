import SwiftUI
import SceneKit
import URDFKit
import simd

/// A SceneKit viewport embedded in SwiftUI.
struct RobotSceneView: NSViewRepresentable {
    @EnvironmentObject var appState: AppState

    // MARK: - NSViewRepresentable

    func makeCoordinator() -> Coordinator { Coordinator() }

    func makeNSView(context: Context) -> SCNView {
        let sv = SCNView()
        sv.scene = context.coordinator.scene
        sv.allowsCameraControl = true
        sv.autoenablesDefaultLighting = true
        sv.showsStatistics = false
        sv.backgroundColor = NSColor(calibratedWhite: 0.12, alpha: 1.0)
        sv.antialiasingMode = .multisampling4X
        // SceneKit renders continuously via its own Metal display link.
        // ALL SceneKit writes happen inside renderer(_:updateAtTime:) on the
        // CVDisplayLink thread — never on the main thread — to avoid deadlocking
        // against SceneKit's internal render lock.
        sv.rendersContinuously = true
        sv.delegate = context.coordinator

        context.coordinator.appState = appState
        context.coordinator.scnView  = sv

        // Mouse handlers for IK gizmo drag.
        // We use NSEvent local monitors so we don't need the view to be first responder.
        let coord = context.coordinator
        NSEvent.addLocalMonitorForEvents(matching: .leftMouseDown) { [weak coord] event in
            coord?.handleMouseDown(event: event); return event
        }
        NSEvent.addLocalMonitorForEvents(matching: .leftMouseDragged) { [weak coord] event in
            coord?.handleMouseDrag(event: event); return event
        }
        NSEvent.addLocalMonitorForEvents(matching: .leftMouseUp) { [weak coord] event in
            coord?.handleMouseUp(event: event); return event
        }
        // Runs in .common mode so it fires even during AppKit event-tracking
        // (sustained slider drag). All SceneKit writes happen here — on the
        // MAIN thread — so no locking, no BG-thread scene-graph races, and
        // no SCNTransaction implicit-animation surprises.
        let c = context.coordinator
        let timer = Timer(timeInterval: 1.0 / 120, repeats: true) { [weak c] _ in
            guard let c, let state = c.appState else { return }

            // Collision check — pure value-type computation, trivially fast (~μs for ≤20 links).
            let collision: CollisionResult
            if let model = state.robotModel, let fk = state.fkResult {
                collision = CollisionChecker.check(model: model, fkResult: fk,
                                                   obstacles: state.obstacles,
                                                   disabledPairs: state.robotSetup.disabledCollisions)
            } else {
                collision = .clear
            }
            // Throttle SwiftUI collision publishes to ≤10 Hz.
            // The snapshot is updated every tick for smooth scene tinting (see below).
            // Writing @Published at 120 Hz causes objectWillChange to fire 120×/s,
            // re-rendering JointSliderPanel and interfering with active slider drags.
            let now = Date()
            if now.timeIntervalSince(c.lastCollisionPublishTime) >= 0.1
               && collision != state.collisionResult {
                state.collisionResult = collision
                c.lastCollisionPublishTime = now
            }

            // IK solve — runs every tick when IK mode is active.
            // Writes directly to state.jointAngles (plain var, no objectWillChange).
            if state.useIK,
               let model = state.robotModel,
               !state.ikEndEffectorLink.isEmpty {
                let result = IKSolver.solve(
                    model: model,
                    endEffectorLinkName: state.ikEndEffectorLink,
                    targetTransform: state.ikTarget,
                    initialAngles: state.jointAngles,
                    maxIterations: 20,   // low per-tick budget; iterates across frames
                    tolerance: 1e-3
                )
                state.jointAngles = result.jointAngles
            }

            c.snapshotLock.lock()
            c.snapshotAngles          = state.jointAngles
            c.snapshotObstacles       = state.obstacles
            c.snapshotRevision        = state.robotModelRevision
            c.snapshotModel           = state.robotModel
            c.snapshotURDFDir         = state.urdfURL?.deletingLastPathComponent()
            c.snapshotCollisionResult = collision
            c.snapshotUseIK           = state.useIK
            c.snapshotIKTarget        = state.ikTarget
            c.snapshotIKLink          = state.ikEndEffectorLink
            c.snapshotTrajectoryPath  = state.trajectoryEEPath
            c.snapshotLock.unlock()
        }
        RunLoop.main.add(timer, forMode: .common)
        context.coordinator.refreshTimer = timer

        return sv
    }

    func updateNSView(_ nsView: SCNView, context: Context) {
        // Keep coordinator's appState ref current in case SwiftUI replaces it.
        context.coordinator.appState = appState
    }

    // MARK: - Coordinator

    final class Coordinator: NSObject, SCNSceneRendererDelegate {

        // ── Conversion: URDF (Z-up / ROS) → SceneKit (Y-up) ─────────────
        // Encodes: URDF X → SCN X,  URDF Y → SCN –Z,  URDF Z → SCN Y
        private static let R: simd_double4x4 = simd_double4x4(columns: (
            SIMD4<Double>(1,  0,  0, 0),
            SIMD4<Double>(0,  0, -1, 0),
            SIMD4<Double>(0,  1,  0, 0),
            SIMD4<Double>(0,  0,  0, 1)
        ))

        /// Convert a URDF-space 4×4 transform to a SceneKit-space float matrix.
        func toSceneKit(_ m: simd_double4x4) -> simd_float4x4 {
            let R = Self.R
            let c = R * m * R.transpose
            func f(_ d: SIMD4<Double>) -> SIMD4<Float> {
                SIMD4<Float>(Float(d.x), Float(d.y), Float(d.z), Float(d.w))
            }
            return simd_float4x4(f(c.columns.0), f(c.columns.1),
                                 f(c.columns.2), f(c.columns.3))
        }

        // ── Scene ─────────────────────────────────────────────────────────
        let scene: SCNScene = {
            let s = SCNScene()

            // Floor — SCNFloor is broken on modern macOS/Metal (FloorPass graph error).
            // Use a large SCNPlane rotated flat instead.
            let plane = SCNPlane(width: 10, height: 10)
            plane.firstMaterial?.diffuse.contents = makeFloorGridTexture()
            plane.firstMaterial?.isDoubleSided = true
            let floorNode = SCNNode(geometry: plane)
            floorNode.eulerAngles.x = -.pi / 2
            s.rootNode.addChildNode(floorNode)

            // World-axis indicator
            s.rootNode.addChildNode(makeAxisNode(length: 0.15))

            // Camera
            let cam = SCNCamera()
            cam.fieldOfView = 45
            cam.zNear = 0.01
            cam.zFar = 100
            let camNode = SCNNode()
            camNode.camera = cam
            camNode.position = SCNVector3(1.0, 1.2, 1.8)
            camNode.look(at: SCNVector3(0, 0.5, 0))
            s.rootNode.addChildNode(camNode)

            return s
        }()

        // Called once after the scene is created
        override init() {
            super.init()
            scene.rootNode.addChildNode(obstacleContainerNode)
            scene.rootNode.addChildNode(trajectoryPathNode)
        }

        // ── AppState reference (main thread only) ────────────────────────────────
        var appState: AppState?
        var refreshTimer: Timer?

        deinit { refreshTimer?.invalidate() }

        // ── Snapshot (written on main thread by timer, read on BG render thread) ────
        let snapshotLock = NSLock()
        var snapshotAngles:          [String: Double] = [:]
        var snapshotObstacles:       [Obstacle] = []
        var snapshotRevision:        Int = -1
        var snapshotModel:           RobotModel? = nil
        var snapshotURDFDir:         URL? = nil
        var snapshotCollisionResult: CollisionResult = .clear
        var snapshotUseIK:           Bool = false
        var snapshotIKTarget:        simd_double4x4 = .identity
        var snapshotIKLink:          String = ""
        var snapshotTrajectoryPath:  [SIMD3<Double>] = []

        // ── Trajectory path node ───────────────────────────────────────────
        private var trajectoryPathNode: SCNNode = {
            let n = SCNNode(); n.name = "trajectoryPath"; return n
        }()

        // ── Logging / throttle tracking ─────────────────────────────────────────
        var lastLogTime:             Date = .distantPast
        var lastCollisionPublishTime: Date = .distantPast

        // ── IK gizmo state ────────────────────────────────────────────────────
        // The gizmo is three axis-arrows anchored at the end-effector.
        // Each arrow is an SCNNode named "gizmo:x", "gizmo:y", "gizmo:z".
        // Dragging an arrow translates the IK target in that axis direction.
        private var gizmoNode: SCNNode?
        private var gizmoVisible: Bool = false
        /// Which axis the user is currently dragging: 0=x, 1=y, 2=z, -1=none
        private var draggingAxis: Int = -1
        /// Screen-space position when the drag started
        private var dragStartScreen: CGPoint = .zero
        /// IK target position when the drag started (URDF frame)
        private var dragStartTargetPos: SIMD3<Double> = .zero

        // ── Obstacle drag state ───────────────────────────────────────────────
        private var draggingObstacleID: UUID?  = nil
        /// SceneKit Y of the horizontal plane on which the obstacle is dragged.
        private var dragObs_planeY:  Float  = 0
        /// Offset (URDF X,Y) from obstacle centre to the initial ray-plane hit.
        private var dragObs_offsetX: Double = 0
        private var dragObs_offsetY: Double = 0
        /// Reference to the SCNView (set in makeNSView, used for hit-testing)
        weak var scnView: SCNView?

        // ── SCNSceneRendererDelegate ──────────────────────────────────────────────
        // Fired on the CVDisplayLink thread. SceneKit already holds its internal
        // render lock here, so addChildNode / simdTransform writes are safe and
        // go directly to the presentation layer (no implicit animation).
        func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
            snapshotLock.lock()
            let angles          = snapshotAngles
            let obstacles       = snapshotObstacles
            let revision        = snapshotRevision
            let model           = snapshotModel
            let urdfDir         = snapshotURDFDir
            let collisionResult = snapshotCollisionResult
            let useIK           = snapshotUseIK
            let ikTarget        = snapshotIKTarget
            let ikLink          = snapshotIKLink
            let trajectoryPath  = snapshotTrajectoryPath
            snapshotLock.unlock()

            // Throttled log — one line per second showing all joint angles.
            if Date().timeIntervalSince(lastLogTime) > 1.0 {
                lastLogTime = Date()
                let names = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                             "wrist_1_joint","wrist_2_joint","wrist_3_joint"]
                let vals = names.map { n -> String in
                    let a = angles[n] ?? 0
                    return "\(n.split(separator:"_").first ?? "?")=\(String(format:"%.2f",a))"
                }.joined(separator: "  ")
                print("[RND] rev=\(revision) nodes=\(jointNodes.count)  \(vals)")
            }

            // Rebuild scene graph when a new URDF is loaded.
            if revision != lastBuiltRevision {
                lastBuiltRevision = revision
                if let model {
                    buildRobotNodes(model: model, urdfDir: urdfDir)
                } else {
                    clearRobotNodes()
                }
            }

            guard let model else { return }

            // No SCNTransaction wrapper here — Apple guarantees that writes made
            // inside renderer(_:updateAtTime:) go directly to the presentation layer
            // without animation. Wrapping in SCNTransaction.begin/commit actually
            // re-introduces the animation pipeline and defeats that guarantee.
            updatePose(model: model, jointAngles: angles)
            updateObstacles(obstacles)
            updateCollisionTint(model: model, collidingLinks: collisionResult.collidingLinks)
            updateGizmo(useIK: useIK, ikTarget: ikTarget, ikLink: ikLink, model: model, angles: angles)
            updateTrajectoryPath(trajectoryPath)
        }

        // ── Robot state ───────────────────────────────────────────────────────
        // linkNodes:  link name  → link-frame SCNNode  (holds geometry children)
        // jointNodes: joint name → joint-offset SCNNode (drives local rotation)
        // Hierarchy: container → rootLinkNode → jointNode → childLinkNode → ...
        // Per-frame update sets only jointNode.simdTransform (local, parent-relative).
        // SceneKit propagates world transforms automatically — no FK required.
        private var linkNodes:       [String: SCNNode] = [:]
        private var linkDefaultColors: [String: NSColor] = [:]
        var jointNodes: [String: SCNNode] = [:]
        private var robotContainerNode: SCNNode?
        var lastBuiltRevision: Int = -1

        // ── Obstacle state ────────────────────────────────────────────────
        private var obstacleNodes: [UUID: SCNNode] = [:]
        private var obstacleContainerNode: SCNNode = {
            let n = SCNNode(); n.name = "obstacles"; return n
        }()

        // MARK: Build

        func buildRobotNodes(model: RobotModel, urdfDir: URL?) {
            clearRobotNodes()
            let container = SCNNode()
            container.name = "robot:\(model.name)"
            scene.rootNode.addChildNode(container)
            robotContainerNode = container

            // Pre-create one link-frame node per link (holds geometry, identity local
            // transform relative to its parent joint node).
            var frameNodes: [String: SCNNode] = [:]
            for (name, link) in model.links {
                let n = makeLinkNode(link: link, urdfDir: urdfDir)
                n.name = "link:\(name)"
                frameNodes[name] = n
                linkNodes[name] = n
            }

            // Capture default material colors for collision tint restore.
            for (name, frameNode) in frameNodes {
                if let visNode = frameNode.childNodes.first,
                   let color = visNode.geometry?.firstMaterial?.diffuse.contents as? NSColor {
                    linkDefaultColors[name] = color
                }
            }

            // Attach root link directly to the container.
            if let root = frameNodes[model.rootLinkName] {
                container.addChildNode(root)
            }

            // BFS: wire parent_link → joint_node → child_link.
            // The joint node carries the joint-origin offset + joint-angle rotation;
            // the child link node stays at identity relative to its joint node.
            var queue = [model.rootLinkName]
            while !queue.isEmpty {
                let parentName = queue.removeFirst()
                guard let parentFrameNode = frameNodes[parentName] else { continue }

                let childJoints = model.joints.values
                    .filter { $0.parentLinkName == parentName }
                for joint in childJoints {
                    let jNode = SCNNode()
                    jNode.name = "joint:\(joint.name)"
                    // Static initial pose = joint origin (angle = 0).
                    jNode.simdTransform = toSceneKit(simd_double4x4.from(transform: joint.origin))
                    parentFrameNode.addChildNode(jNode)
                    jointNodes[joint.name] = jNode

                    if let childFrameNode = frameNodes[joint.childLinkName] {
                        childFrameNode.simdTransform = matrix_identity_float4x4
                        jNode.addChildNode(childFrameNode)
                    }
                    queue.append(joint.childLinkName)
                }
            }
        }

        func clearRobotNodes() {
            robotContainerNode?.removeFromParentNode()
            robotContainerNode = nil
            linkNodes         = [:]
            linkDefaultColors = [:]
            jointNodes        = [:]
            gizmoNode?.removeFromParentNode()
            gizmoNode = nil
        }

        // MARK: Trajectory Path

        /// Rebuild the EE polyline whenever the sampled path changes.
        func updateTrajectoryPath(_ eePoints: [SIMD3<Double>]) {
            // Remove previous path segments.
            trajectoryPathNode.childNodes.forEach { $0.removeFromParentNode() }

            guard eePoints.count >= 2 else {
                if trajectoryPathNode.parent == nil {
                    scene.rootNode.addChildNode(trajectoryPathNode)
                }
                return
            }

            if trajectoryPathNode.parent == nil {
                scene.rootNode.addChildNode(trajectoryPathNode)
            }

            let color = NSColor.systemCyan.withAlphaComponent(0.85)
            let tubeRadius: CGFloat = 0.004

            // Convert URDF-frame points to SceneKit frame (Z-up → Y-up).
            // URDF X→SCN X, URDF Y→SCN -Z, URDF Z→SCN Y
            func scnPos(_ p: SIMD3<Double>) -> SCNVector3 {
                SCNVector3(Float(p.x), Float(p.z), Float(-p.y))
            }

            for i in 0 ..< eePoints.count - 1 {
                let a = scnPos(eePoints[i])
                let b = scnPos(eePoints[i + 1])
                let dx = Float(b.x) - Float(a.x)
                let dy = Float(b.y) - Float(a.y)
                let dz = Float(b.z) - Float(a.z)
                let segLen = CGFloat(sqrt(dx*dx + dy*dy + dz*dz))
                guard segLen > 1e-5 else { continue }

                let tube = SCNCylinder(radius: tubeRadius, height: segLen)
                tube.firstMaterial?.diffuse.contents = color
                tube.firstMaterial?.emission.contents = color

                let seg = SCNNode(geometry: tube)
                // Position at midpoint.
                seg.position = SCNVector3(Float(a.x) + dx/2, Float(a.y) + dy/2, Float(a.z) + dz/2)
                // Orient along segment direction.
                // SCNCylinder's axis is Y; we rotate Y to the direction vector.
                let dir = simd_normalize(SIMD3<Float>(dx, dy, dz))
                let yAxis = SIMD3<Float>(0, 1, 0)
                let cross = simd_cross(yAxis, dir)
                let dot   = simd_dot(yAxis, dir)
                if simd_length(cross) < 1e-6 {
                    // Parallel — no rotation needed (or flip 180°).
                    if dot < 0 { seg.eulerAngles = SCNVector3(Float.pi, 0, 0) }
                } else {
                    let angle = acos(max(-1, min(1, dot)))
                    seg.simdOrientation = simd_quaternion(angle, simd_normalize(cross))
                }
                trajectoryPathNode.addChildNode(seg)
            }

            // Draw sphere markers at start and goal.
            for (idx, point) in [eePoints.first!, eePoints.last!].enumerated() {
                let sphere = SCNSphere(radius: tubeRadius * 3)
                sphere.firstMaterial?.diffuse.contents = idx == 0
                    ? NSColor.systemGreen.withAlphaComponent(0.9)
                    : NSColor.systemRed.withAlphaComponent(0.9)
                sphere.firstMaterial?.emission.contents = sphere.firstMaterial?.diffuse.contents
                let node = SCNNode(geometry: sphere)
                node.position = scnPos(point)
                trajectoryPathNode.addChildNode(node)
            }
        }

        // MARK: IK Gizmo

        /// Show / hide and reposition the gizmo each render frame.
        func updateGizmo(
            useIK: Bool,
            ikTarget: simd_double4x4,
            ikLink: String,
            model: RobotModel,
            angles: [String: Double]
        ) {
            if !useIK {
                if gizmoVisible { gizmoNode?.isHidden = true; gizmoVisible = false }
                return
            }

            // Build the gizmo once.
            if gizmoNode == nil { buildGizmo() }
            guard let gizmo = gizmoNode else { return }

            if gizmoVisible == false { gizmo.isHidden = false; gizmoVisible = true }

            // Position the gizmo at the IK target in SceneKit space.
            // ikTarget is in URDF (Z-up) frame; convert to SCN (Y-up).
            gizmo.simdTransform = toSceneKit(ikTarget)
        }

        private func buildGizmo() {
            let root = SCNNode()
            root.name = "ik_gizmo"

            let length: CGFloat = 0.12
            let radius: CGFloat = 0.008

            func axisShaft(color: NSColor, axis: Int) -> SCNNode {
                let cyl = SCNCylinder(radius: radius, height: length)
                cyl.firstMaterial?.diffuse.contents = color
                cyl.firstMaterial?.emission.contents = color.withAlphaComponent(0.5)
                let shaft = SCNNode(geometry: cyl)

                let cone = SCNCone(topRadius: 0, bottomRadius: radius * 2.5, height: radius * 5)
                cone.firstMaterial?.diffuse.contents = color
                cone.firstMaterial?.emission.contents = color.withAlphaComponent(0.5)
                let tip = SCNNode(geometry: cone)
                tip.position = SCNVector3(0, Float(length / 2 + radius * 2.5), 0)
                shaft.addChildNode(tip)

                let wrapper = SCNNode()
                wrapper.name = "gizmo:\(axis)"
                switch axis {
                case 0: wrapper.eulerAngles = SCNVector3(0, 0, -Float.pi / 2)
                case 2: wrapper.eulerAngles = SCNVector3(Float.pi / 2, 0, 0)
                default: break
                }
                shaft.position = SCNVector3(0, Float(length / 2), 0)
                wrapper.addChildNode(shaft)
                return wrapper
            }

            root.addChildNode(axisShaft(color: .red,   axis: 0))
            root.addChildNode(axisShaft(color: .green, axis: 1))
            root.addChildNode(axisShaft(color: .blue,  axis: 2))

            scene.rootNode.addChildNode(root)
            gizmoNode = root
            root.isHidden = true
        }

        // MARK: Mouse events (IK gizmo drag + obstacle drag)

        func handleMouseDown(event: NSEvent) {
            guard let sv = scnView, let state = appState else { return }
            let loc = sv.convert(event.locationInWindow, from: nil)
            let hits = sv.hitTest(loc, options: [.searchMode: SCNHitTestSearchMode.all.rawValue])

            // ── 1. IK gizmo (only when IK is active) ────────────────────────
            if state.useIK {
                for hit in hits {
                    var n: SCNNode? = hit.node
                    while let node = n {
                        if let name = node.name, name.hasPrefix("gizmo:"),
                           let axisChar = name.last, let axis = Int(String(axisChar)) {
                            draggingAxis = axis
                            dragStartScreen = loc
                            dragStartTargetPos = state.ikTarget.translation
                            return
                        }
                        n = node.parent
                    }
                }
            }
            draggingAxis = -1

            // ── 2. Obstacle grab ─────────────────────────────────────────────
            for hit in hits {
                var n: SCNNode? = hit.node
                while let node = n {
                    if let name = node.name, name.hasPrefix("obstacle:"),
                       let uuidStr = name.components(separatedBy: ":").last,
                       let uuid = UUID(uuidString: uuidStr),
                       let idx = state.obstacles.firstIndex(where: { $0.id == uuid }) {
                        let obs = state.obstacles[idx]
                        // Horizontal drag plane at the obstacle's SceneKit Y (= URDF Z)
                        let planeY = Float(obs.position.z)
                        draggingObstacleID = uuid
                        dragObs_planeY     = planeY
                        if let hp = rayPlaneIntersect(screenPt: loc, planeY: planeY, in: sv) {
                            dragObs_offsetX = Double(hp.x) - obs.position.x
                            dragObs_offsetY = -Double(hp.z) - obs.position.y
                        } else {
                            dragObs_offsetX = 0; dragObs_offsetY = 0
                        }
                        // Brighten obstacle while dragging
                        obstacleNodes[uuid]?.geometry?.firstMaterial?.diffuse.contents =
                            NSColor.white.withAlphaComponent(0.9)
                        sv.allowsCameraControl = false
                        return
                    }
                    n = node.parent
                }
            }
        }

        func handleMouseDrag(event: NSEvent) {
            guard let sv = scnView, let state = appState else { return }
            let loc = sv.convert(event.locationInWindow, from: nil)

            // ── Obstacle drag ────────────────────────────────────────────────
            if let obsID = draggingObstacleID {
                if let hp = rayPlaneIntersect(screenPt: loc, planeY: dragObs_planeY, in: sv),
                   let idx = state.obstacles.firstIndex(where: { $0.id == obsID }) {
                    state.obstacles[idx].position.x = Double(hp.x) - dragObs_offsetX
                    state.obstacles[idx].position.y = -Double(hp.z) - dragObs_offsetY
                }
                return
            }

            // ── IK gizmo drag ────────────────────────────────────────────────
            guard draggingAxis >= 0 else { return }

            let dx = Double(loc.x - dragStartScreen.x)
            let dy = Double(loc.y - dragStartScreen.y)

            // Scale screen pixels → metres. ~200 px per metre feels responsive.
            let scale = 1.0 / 200.0
            var delta = SIMD3<Double>.zero
            switch draggingAxis {
            case 0: delta = SIMD3(dx * scale, 0, 0)
            case 1: delta = SIMD3(0, 0, dy * scale)   // URDF Z = up
            case 2: delta = SIMD3(0, -dx * scale, 0)  // URDF Y = screen horizontal
            default: return
            }

            var newTarget = state.ikTarget
            newTarget.columns.3 = SIMD4(
                dragStartTargetPos.x + delta.x,
                dragStartTargetPos.y + delta.y,
                dragStartTargetPos.z + delta.z,
                1
            )
            state.ikTarget = newTarget
        }

        func handleMouseUp(event: NSEvent) {
            if let obsID = draggingObstacleID {
                // Restore obstacle colour
                obstacleNodes[obsID]?.geometry?.firstMaterial?.diffuse.contents =
                    NSColor.systemOrange.withAlphaComponent(0.65)
                scnView?.allowsCameraControl = true
                draggingObstacleID = nil
            }
            draggingAxis = -1
        }

        /// Project a screen point to 3-D by intersecting the camera ray with a
        /// horizontal (constant SceneKit-Y) plane. Returns nil when the ray is
        /// nearly parallel to the plane.
        private func rayPlaneIntersect(screenPt: CGPoint,
                                       planeY: Float,
                                       in sv: SCNView) -> SIMD3<Float>? {
            let near = sv.unprojectPoint(SCNVector3(screenPt.x, screenPt.y, 0))
            let far  = sv.unprojectPoint(SCNVector3(screenPt.x, screenPt.y, 1))
            let dy = Float(far.y - near.y)
            guard abs(dy) > 1e-6 else { return nil }
            let t = (planeY - Float(near.y)) / dy
            return SIMD3<Float>(Float(near.x) + t * Float(far.x - near.x),
                                planeY,
                                Float(near.z) + t * Float(far.z - near.z))
        }

        // MARK: Update

        func updateObstacles(_ obstacles: [Obstacle]) {
            let current = Set(obstacles.map { $0.id })

            // Remove deleted obstacles
            for id in Set(obstacleNodes.keys).subtracting(current) {
                obstacleNodes[id]?.removeFromParentNode()
                obstacleNodes.removeValue(forKey: id)
            }

            for obstacle in obstacles {
                let node: SCNNode
                if let existing = obstacleNodes[obstacle.id] {
                    node = existing
                } else {
                    node = makeObstacleNode(obstacle)
                    obstacleContainerNode.addChildNode(node)
                    obstacleNodes[obstacle.id] = node
                }
                // Position: convert URDF Z-up to SceneKit Y-up
                let p = obstacle.position
                node.simdPosition = SIMD3<Float>(Float(p.x), Float(p.z), Float(-p.y))
            }
        }

        private func makeObstacleNode(_ obstacle: Obstacle) -> SCNNode {
            let geom: SCNGeometry
            switch obstacle.shape {
            case .box:
                let s = obstacle.boxSize
                // URDF (x,y,z) → SCN (width=x, height=z, length=y)
                geom = SCNBox(width: CGFloat(s.x), height: CGFloat(s.z),
                              length: CGFloat(s.y), chamferRadius: 0)
            case .cylinder:
                geom = SCNCylinder(radius: CGFloat(obstacle.radius),
                                   height: CGFloat(obstacle.height))
            case .sphere:
                geom = SCNSphere(radius: CGFloat(obstacle.radius))
            }
            geom.firstMaterial?.diffuse.contents = NSColor.systemOrange.withAlphaComponent(0.65)
            geom.firstMaterial?.transparency = 0.65
            let node = SCNNode(geometry: geom)
            node.name = "obstacle:\(obstacle.id)"
            return node
        }

        /// Update the kinematic pose by writing only joint-local transforms.
        /// SceneKit propagates the changes down the parent-child hierarchy automatically.
        /// Must be called from renderer(_:updateAtTime:) so writes go directly to the
        /// presentation layer, bypassing SceneKit's implicit transaction animations.
        func updatePose(model: RobotModel, jointAngles: [String: Double]) {
            for (jName, jNode) in jointNodes {
                guard let joint = model.joints[jName] else { continue }
                let angle = jointAngles[jName] ?? 0.0
                let T: simd_double4x4
                switch joint.type {
                case .revolute, .continuous:
                    T = simd_double4x4.from(transform: joint.origin)
                        * .rotation(angle: angle, axis: joint.axis)
                case .prismatic:
                    T = simd_double4x4.from(transform: joint.origin)
                        * .translation(joint.axis * angle)
                default:
                    T = simd_double4x4.from(transform: joint.origin)
                }
                jNode.simdTransform = toSceneKit(T)
            }
        }

        /// Tint links red when colliding; restore default color when clear.
        /// Called from renderer(_:updateAtTime:) so material writes are on the render thread.
        func updateCollisionTint(model: RobotModel, collidingLinks: Set<String>) {
            for (name, linkNode) in linkNodes {
                let color: NSColor = collidingLinks.contains(name)
                    ? .systemRed.withAlphaComponent(0.8)
                    : (linkDefaultColors[name] ?? defaultLinkColor(model.links[name]))
                setColor(linkNode, color: color)
            }
        }

        private func defaultLinkColor(_ link: URDFKit.Link?) -> NSColor {
            switch link?.visual?.geometry {
            case .box:      return .systemBlue.withAlphaComponent(0.7)
            case .cylinder: return .systemTeal.withAlphaComponent(0.7)
            case .sphere:   return .systemGreen.withAlphaComponent(0.7)
            default:        return .gray.withAlphaComponent(0.7)
            }
        }

        /// Recursively set diffuse color on all geometry-bearing nodes in the subtree.
        private func setColor(_ node: SCNNode, color: NSColor) {
            node.geometry?.firstMaterial?.diffuse.contents = color
            for child in node.childNodes { setColor(child, color: color) }
        }

        // MARK: Node factories

        private func makeLinkNode(link: URDFKit.Link, urdfDir: URL?) -> SCNNode {
            let node = SCNNode()
            if let visual = link.visual {
                let visNode = makeGeomNode(visual.geometry, urdfDir: urdfDir)
                // Visual origin is in URDF link-frame coordinates; convert to SCN frame.
                visNode.simdTransform = toSceneKit(
                    simd_double4x4.from(transform: visual.origin)
                )
                node.addChildNode(visNode)
            } else {
                node.addChildNode(Self.makeAxisNode(length: 0.05))
            }
            return node
        }

        private func makeGeomNode(_ geom: Geometry, urdfDir: URL?) -> SCNNode {
            switch geom {
            case .box(let size):
                // Map URDF (x, y, z) → SCN (width=x, height=z_up, length=y)
                let g = SCNBox(
                    width:  CGFloat(size.x),
                    height: CGFloat(size.z),
                    length: CGFloat(size.y),
                    chamferRadius: 0
                )
                g.firstMaterial?.diffuse.contents = NSColor.systemBlue.withAlphaComponent(0.7)
                return SCNNode(geometry: g)

            case .cylinder(let radius, let length):
                // URDF cylinder axis = URDF Z → SCN Y (SCNCylinder default). ✓
                let g = SCNCylinder(radius: CGFloat(radius), height: CGFloat(length))
                g.firstMaterial?.diffuse.contents = NSColor.systemTeal.withAlphaComponent(0.7)
                return SCNNode(geometry: g)

            case .sphere(let radius):
                let g = SCNSphere(radius: CGFloat(radius))
                g.firstMaterial?.diffuse.contents = NSColor.systemGreen.withAlphaComponent(0.7)
                return SCNNode(geometry: g)

            case .mesh(let filename, let scale):
                return loadMeshNode(filename: filename, scale: scale, urdfDir: urdfDir)
            }
        }

        private func loadMeshNode(
            filename: String,
            scale: SIMD3<Double>,
            urdfDir: URL?
        ) -> SCNNode {
            let wrapper = SCNNode()
            wrapper.simdScale = SIMD3<Float>(Float(scale.x), Float(scale.y), Float(scale.z))

            guard let resolvedURL = resolveMeshURL(filename, urdfDir: urdfDir) else {
                wrapper.addChildNode(fallbackNode())
                return wrapper
            }

            // SCNScene(url:options:) handles .dae, .obj, and delegates to
            // ModelIO for .stl/.ply. Fall back to a placeholder sphere on error.
            if let scn = try? SCNScene(url: resolvedURL, options: nil) {
                let children = scn.rootNode.childNodes
                if children.isEmpty {
                    wrapper.addChildNode(fallbackNode())
                } else {
                    for child in children { wrapper.addChildNode(child.clone()) }
                }
            } else {
                wrapper.addChildNode(fallbackNode())
            }
            return wrapper
        }

        private func resolveMeshURL(_ path: String, urdfDir: URL?) -> URL? {
            guard let base = urdfDir else { return nil }

            // Strip "package://<pkg_name>/" prefix from ROS package paths
            var rel = path
            if rel.hasPrefix("package://") {
                rel = String(rel.dropFirst("package://".count))
                if let slash = rel.firstIndex(of: "/") {
                    rel = String(rel[rel.index(after: slash)...])
                }
            }

            // 1. Try base/<relative_path>
            let candidate = base.appendingPathComponent(rel)
            if FileManager.default.fileExists(atPath: candidate.path) { return candidate }

            // 2. Try base/<basename> (last path component)
            let basename = (rel as NSString).lastPathComponent
            let candidate2 = base.appendingPathComponent(basename)
            if FileManager.default.fileExists(atPath: candidate2.path) { return candidate2 }

            return nil
        }

        private func fallbackNode() -> SCNNode {
            let g = SCNSphere(radius: 0.02)
            g.firstMaterial?.diffuse.contents = NSColor.gray
            return SCNNode(geometry: g)
        }

        // MARK: Shared helpers

        private static func makeAxisNode(length: Float) -> SCNNode {
            let root = SCNNode()

            func arrow(color: NSColor, eulerAngles: SCNVector3, offset: SCNVector3) -> SCNNode {
                let cyl = SCNCylinder(radius: 0.004, height: CGFloat(length))
                cyl.firstMaterial?.diffuse.contents = color
                let n = SCNNode(geometry: cyl)
                n.eulerAngles = eulerAngles
                n.position = offset
                return n
            }

            let h = length / 2
            root.addChildNode(arrow(color: .red,
                eulerAngles: SCNVector3(0, 0, -Float.pi / 2),
                offset: SCNVector3(h, 0, 0)))
            root.addChildNode(arrow(color: .green,
                eulerAngles: .init(0, 0, 0),
                offset: SCNVector3(0, h, 0)))
            root.addChildNode(arrow(color: .blue,
                eulerAngles: SCNVector3(Float.pi / 2, 0, 0),
                offset: SCNVector3(0, 0, h)))
            return root
        }
    }
}

// MARK: - Floor grid texture

/// Draws a dark floor tile with a white grid overlay.
private func makeFloorGridTexture(textureSize: Int = 1024, cells: Int = 10) -> NSImage {
    let img = NSImage(size: NSSize(width: textureSize, height: textureSize))
    img.lockFocus()
    defer { img.unlockFocus() }

    // Background
    NSColor(calibratedWhite: 0.18, alpha: 1).setFill()
    NSRect(x: 0, y: 0, width: textureSize, height: textureSize).fill()

    // Grid lines
    let path = NSBezierPath()
    path.lineWidth = 1.5
    NSColor(white: 1.0, alpha: 0.35).setStroke()
    let step = CGFloat(textureSize) / CGFloat(cells)
    for i in 0...cells {
        let p = CGFloat(i) * step
        // vertical
        path.move(to: CGPoint(x: p, y: 0))
        path.line(to: CGPoint(x: p, y: CGFloat(textureSize)))
        // horizontal
        path.move(to: CGPoint(x: 0, y: p))
        path.line(to: CGPoint(x: CGFloat(textureSize), y: p))
    }
    path.stroke()

    return img
}
