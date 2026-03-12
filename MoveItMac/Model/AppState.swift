import Foundation
import SwiftUI
import simd
import URDFKit

// MARK: - Obstacle model

enum ObstacleShape: String, CaseIterable, Identifiable, Codable {
    case box, cylinder, sphere
    var id: String { rawValue }
    var label: String { rawValue.capitalized }
    var systemImage: String {
        switch self {
        case .box:      return "cube"
        case .cylinder: return "cylinder"
        case .sphere:   return "circle.circle"
        }
    }
}

struct Obstacle: Identifiable, Codable {
    let id: UUID
    var name: String
    var shape: ObstacleShape
    var position: SIMD3<Double>
    var boxSize: SIMD3<Double>   // used when shape == .box
    var radius: Double           // used when shape == .cylinder | .sphere
    var height: Double           // used when shape == .cylinder

    init(
        id: UUID = UUID(),
        name: String,
        shape: ObstacleShape,
        position: SIMD3<Double>,
        boxSize: SIMD3<Double> = SIMD3(0.1, 0.1, 0.1),
        radius: Double = 0.05,
        height: Double = 0.1
    ) {
        self.id = id; self.name = name; self.shape = shape
        self.position = position; self.boxSize = boxSize
        self.radius = radius; self.height = height
    }
}

// MARK: - App State

/// Central model object for the MoveIt Mac application.
/// All mutation must occur on the main thread.
final class AppState: ObservableObject {

    // ── Robot ──────────────────────────────────────────────────────────────
    @Published var robotModel: RobotModel? = nil
    /// Joint angles are written at up to 120 Hz by slider drag — NOT @Published.
    /// Making this @Published would fire objectWillChange 120×/s, causing SwiftUI
    /// to re-render JointSliderPanel and call updateNSView on every ContinuousNSSlider
    /// mid-drag, which fights AppKit's own mouse-tracking event loop and freezes joints.
    /// The 3-D view reads this directly via the 120 Hz timer (not through SwiftUI).
    /// UI components that need to display angles use local @State fed by slider callbacks.
    var jointAngles: [String: Double] = [:]
    /// Monotonically-increasing counter so RobotSceneView can detect model changes.
    @Published private(set) var robotModelRevision: Int = 0
    /// URL of the loaded URDF file; used by RobotSceneView to resolve mesh paths.
    @Published private(set) var urdfURL: URL? = nil
    /// Recomputed lazily on every access (fast for ≤ ~20 joints).
    var fkResult: FKResult? {
        guard let model = robotModel else { return nil }
        return ForwardKinematics.compute(model: model, jointAngles: jointAngles)
    }

    var robotName: String? { robotModel?.name }
    var isRobotLoaded: Bool { robotModel != nil }

    /// Joint names in kinematic order (root → leaves).
    var orderedJointNames: [String] {
        robotModel?.orderedActuatedJointNames ?? []
    }

    /// Lower/upper limits for each joint (radians or metres).
    /// Physical center pose captured from the arm's mechanical centering marks.
    /// Nil until the user captures it with "Capture from Arm".
    @Published var centerAngles: [String: Double]? = nil

    var jointLimits: [String: ClosedRange<Double>] {
        guard let model = robotModel else { return [:] }
        return model.joints.values
            .filter { $0.isActuated }
            .reduce(into: [:]) { dict, joint in
                if let lim = joint.limits {
                    dict[joint.name] = lim.lower...lim.upper
                } else {
                    dict[joint.name] = -.pi ... .pi
                }
            }
    }

    // ── Obstacles ──────────────────────────────────────────────────────────
    @Published var obstacles: [Obstacle] = []

    // ── Setup (planning groups, ACM) ──────────────────────────────────────
    @Published var robotSetup: RobotSetup = .empty

    // ── Collision ─────────────────────────────────────────────────────────
    /// Updated at ~120 Hz by the scene-view timer; publishes only when the
    /// set of colliding links actually changes (cheap equality guard).
    @Published var collisionResult: CollisionResult = .clear

    // ── IK ────────────────────────────────────────────────────────────────
    /// Whether the IK gizmo is active. When true, the timer runs IKSolver
    /// instead of letting the sliders drive joint angles directly.
    @Published var useIK: Bool = false
    /// The link that the IK solver should try to place at `ikTarget`.
    @Published var ikEndEffectorLink: String = ""
    /// Desired world-frame end-effector pose (URDF/ROS Z-up frame).
    /// Written by the gizmo drag handler; read by the 120 Hz timer.
    var ikTarget: simd_double4x4 = .identity

    // ── Planner ───────────────────────────────────────────────────────────

    enum PlannerStatus: Equatable {
        case idle, planning, done, failed
    }

    @Published var plannerStatus:  PlannerStatus = .idle
    @Published var plannerMessage: String         = ""
    /// RRT trajectory: sequence of joint-angle configs from start → goal.
    /// Not @Published — read directly by the playback timer.
    var trajectory: [[String: Double]] = []
    /// Time-parameterized version of `trajectory`; set after planning succeeds.
    var timedTrajectory: TimedTrajectory? = nil

    /// Captured start / goal configurations for the planner.
    var planStart: [String: Double] = [:]
    var planGoal:  [String: Double] = [:]

    var canPlan: Bool {
        isRobotLoaded && !planGoal.isEmpty && plannerStatus != .planning
    }

    func setPlanStart() {
        planStart = jointAngles
        plannerMessage = "Start captured"
    }

    func setPlanGoal() {
        planGoal = jointAngles
        plannerMessage = "Goal captured"
    }

    func startPlanning() {
        guard let model = robotModel, canPlan else { return }
        plannerStatus  = .planning
        plannerMessage = "Planning…"
        trajectory     = []

        let startCfg  = planStart.isEmpty ? jointAngles : planStart
        let goalCfg   = planGoal
        let obsCopy   = obstacles
        let setupCopy = robotSetup
        // If an active planning group is configured, plan only over its joints.
        let groupJoints: [String]? = setupCopy.activeGroup.map { $0.joints }

        planningTask?.cancel()
        planningTask = Task.detached(priority: .userInitiated) { [weak self] in
            let result = RRTPlanner.plan(
                model:  model,
                start:  startCfg,
                goal:   goalCfg,
                joints: groupJoints,
                isCollisionFree: { angles in
                    let fk = ForwardKinematics.compute(model: model, jointAngles: angles)
                    let cr = CollisionChecker.check(model: model, fkResult: fk,
                                                    obstacles: obsCopy,
                                                    disabledPairs: setupCopy.disabledCollisions)
                    return !cr.hasCollision
                }
            )
            // Time-parameterize on the background thread before returning.
            let timed: TimedTrajectory? = result.success
                ? TimeParameterization.apply(path: result.path, model: model)
                : nil
            await MainActor.run { [weak self] in
                guard let self else { return }
                self.trajectory        = result.path
                self.timedTrajectory   = timed
                self.plannerStatus     = result.success ? .done : .failed
                let dur = timed.map { String(format: "  (%.1f s)", $0.duration) } ?? ""
                self.plannerMessage    = result.success
                    ? "Found path: \(result.path.count) waypoints\(dur)"
                    : "No path found (\(result.iterations) iters)"
                // Compute sampled EE path for 3-D visualization (up to 80 points).
                if result.success, let model = self.robotModel {
                    let tipLink = self.robotSetup.activeGroup?.tipLink
                        ?? model.orderedActuatedJointNames.last.flatMap { model.joints[$0]?.childLinkName }
                        ?? model.rootLinkName
                    let path = result.path
                    let step = max(1, path.count / 80)
                    var ee: [SIMD3<Double>] = []
                    for i in stride(from: 0, to: path.count, by: step) {
                        let fk = ForwardKinematics.compute(model: model, jointAngles: path[i])
                        ee.append(fk.position(of: tipLink))
                    }
                    if let last = path.last {
                        let fk = ForwardKinematics.compute(model: model, jointAngles: last)
                        ee.append(fk.position(of: tipLink))
                    }
                    self.trajectoryEEPath = ee
                } else {
                    self.trajectoryEEPath = []
                }
            }
        }
    }

    /// Sampled end-effector world positions along the planned trajectory (URDF frame).
    /// Updated whenever `trajectory` changes. Read by RobotSceneView for path rendering.
    @Published var trajectoryEEPath: [SIMD3<Double>] = []

    private var planningTask: Task<Void, Never>?

    // ── Trajectory Execution ──────────────────────────────────────────────

    @Published var isExecuting: Bool = false

    /// Stream the current timed trajectory to the physical arm at ~10 Hz,
    /// while also advancing the virtual arm and scrubber in real time.
    func executeTrajectory(bridge: MyCobotBridge) {
        guard let timed = timedTrajectory, timed.duration > 0,
              let model = robotModel else { return }
        isPlaying  = false
        stopPlaybackTimer()
        playbackProgress = 0
        isExecuting = true

        let names    = Array(model.orderedActuatedJointNames.prefix(6))
        let duration = timed.duration

        Task { @MainActor [weak self] in
            guard let self else { return }
            guard bridge.isConnected else { self.isExecuting = false; return }
            let stepSeconds = 0.10   // 10 Hz
            var t = 0.0
            while t <= duration, self.isExecuting {
                self.playbackProgress = min(t / duration, 1.0)
                let a = timed.angles(at: t)
                self.jointAngles = a
                let angles6 = names.map { a[$0] ?? 0.0 }
                for (i, angle) in angles6.enumerated() {
                    bridge.sendAngle(jointIndex: i, radians: angle, robotSpeed: 80)
                }
                try? await Task.sleep(nanoseconds: UInt64(stepSeconds * 1_000_000_000))
                t += stepSeconds
            }
            // Final frame at exactly t = duration.
            self.playbackProgress = 1.0
            let finalA = timed.angles(at: duration)
            self.jointAngles = finalA
            for (i, angle) in names.map({ finalA[$0] ?? 0.0 }).enumerated() {
                bridge.sendAngle(jointIndex: i, radians: angle, robotSpeed: 80)
            }
            self.isExecuting = false
        }
    }

    // ── Trajectory Playback ───────────────────────────────────────────────

    @Published var isPlaying:         Bool    = false
    /// Current scrubber position 0.0–1.0 (driven by timer or user drag).
    @Published var playbackProgress:  Double  = 0.0
    /// Playback speed: waypoints per second.
    @Published var playbackSpeed:     Double  = 5.0

    var canPlay: Bool { !trajectory.isEmpty }

    /// Index into `trajectory` corresponding to `playbackProgress`.
    var playbackIndex: Int {
        guard trajectory.count > 1 else { return 0 }
        let i = Int((playbackProgress * Double(trajectory.count - 1)).rounded())
        return max(0, min(i, trajectory.count - 1))
    }

    func playPause() {
        guard canPlay else { return }
        if playbackProgress >= 1.0 { playbackProgress = 0.0 }
        isPlaying.toggle()
        if isPlaying { startPlaybackTimer() } else { stopPlaybackTimer() }
    }

    func scrubTo(_ progress: Double) {
        playbackProgress = max(0, min(1, progress))
        applyTrajectoryFrame()
    }

    private func applyTrajectoryFrame() {
        if let timed = timedTrajectory {
            let t = playbackProgress * timed.duration
            jointAngles = timed.angles(at: t)
        } else if !trajectory.isEmpty {
            jointAngles = trajectory[playbackIndex]
        }
    }

    private var playbackTimer: Timer?
    private var playbackLastFire: Date = .now

    private func startPlaybackTimer() {
        playbackTimer?.invalidate()
        playbackLastFire = .now
        playbackTimer = Timer.scheduledTimer(withTimeInterval: 1.0 / 60.0, repeats: true) { [weak self] _ in
            guard let self, self.isPlaying else { return }
            let now       = Date.now
            let elapsed   = now.timeIntervalSince(self.playbackLastFire)
            self.playbackLastFire = now

            if let timed = self.timedTrajectory, timed.duration > 0 {
                // Advance in real time, scaled by playbackSpeed.
                let step = elapsed * self.playbackSpeed / timed.duration
                self.playbackProgress += step
            } else {
                // Fallback: uniform step over waypoints.
                let count = max(self.trajectory.count - 1, 1)
                self.playbackProgress += elapsed * self.playbackSpeed / Double(count)
            }

            if self.playbackProgress >= 1.0 {
                self.playbackProgress = 1.0
                self.isPlaying = false
                self.stopPlaybackTimer()
            }
            self.applyTrajectoryFrame()
        }
    }

    private func stopPlaybackTimer() {
        playbackTimer?.invalidate()
        playbackTimer = nil
    }

    // MARK: - Actions

    /// Opens a file-open panel and loads the selected URDF.
    func openURDFPanel() {
        let panel = NSOpenPanel()
        panel.title = "Open URDF"
        panel.allowedContentTypes = [.init(filenameExtension: "urdf")!]
        panel.allowsMultipleSelection = false
        panel.canChooseDirectories = false

        guard panel.runModal() == .OK, let url = panel.url else { return }
        loadURDF(from: url)
    }

    func loadURDF(from url: URL) {
        do {
            let model = try URDFParser.parse(contentsOf: url)
            urdfURL = url
            setRobotModel(model)
        } catch {
            // Phase 2+: present an alert here
            print("[AppState] Failed to load URDF: \(error.localizedDescription)")
        }
    }

    /// Reload the URDF from the last opened URL (picks up on-disk edits without re-selecting the file).
    func reloadURDF() {
        guard let url = urdfURL else { return }
        loadURDF(from: url)
    }

    // MARK: - Private

    private func setRobotModel(_ model: RobotModel) {
        robotModel = model
        robotModelRevision += 1
        robotSetup = .empty   // clear previous setup when a new robot is loaded
        trajectory = []; timedTrajectory = nil; trajectoryEEPath = []
        plannerStatus = .idle; plannerMessage = ""; planStart = [:]; planGoal = [:]
        // Auto-generate the ACM in the background so jogging doesn't halt on false positives.
        Task.detached(priority: .utility) { [weak self] in
            let pairs = ACMGenerator.compute(model: model)
            await MainActor.run { self?.robotSetup.disabledCollisions = pairs }
        }
        // Initialise all actuated joints to their midpoint (or 0 if no limits)
        jointAngles = model.joints.values
            .filter { $0.isActuated }
            .reduce(into: [:]) { dict, joint in
                if let lim = joint.limits {
                    dict[joint.name] = (lim.lower + lim.upper) / 2
                } else {
                    dict[joint.name] = 0
                }
            }
        // Default end-effector = last link in BFS order
        let bfsLinks: [String] = {
            var result: [String] = [model.rootLinkName]
            var queue = [model.rootLinkName]
            while !queue.isEmpty {
                let cur = queue.removeFirst()
                let children = model.joints.values
                    .filter { $0.parentLinkName == cur }
                    .sorted { $0.name < $1.name }
                    .map { $0.childLinkName }
                result.append(contentsOf: children)
                queue.append(contentsOf: children)
            }
            return result
        }()
        ikEndEffectorLink = bfsLinks.last ?? model.rootLinkName
        // Seed IK target from the current FK pose
        if let fk = fkResult {
            ikTarget = fk.transform(for: ikEndEffectorLink)
        }
        useIK = false
    }
}
