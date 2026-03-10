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

    // MARK: - Private

    private func setRobotModel(_ model: RobotModel) {
        robotModel = model
        robotModelRevision += 1
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
