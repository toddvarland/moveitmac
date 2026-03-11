import Foundation
import simd
import URDFKit

// MARK: - Servo Engine

/// Real-time joint-space and Cartesian jogging engine (MoveIt Servo equivalent).
///
/// A 100 Hz timer applies commanded velocities one step at a time and runs a
/// bounding-sphere collision check before committing each increment.  If a
/// collision is detected the engine halts and the user must press Resume or Stop.
///
/// - All public methods must be called on the main actor.
/// - `appState.jointAngles` is written directly (not via @Published) to remain
///   consistent with the pattern used by the slider panel.
@MainActor
final class ServoEngine: ObservableObject {

    // MARK: - Types

    enum Mode: String, CaseIterable {
        case joint      = "Joint"
        case cartesian  = "Cartesian"
    }

    enum Status: Equatable {
        case idle
        case running
        case halted   // stopped due to imminent collision
    }

    // MARK: - Published state

    @Published private(set) var status: Status = .idle
    @Published var mode: Mode = .joint
    /// Speed multiplier (0.1 – 2.0) applied to all velocity magnitudes.
    @Published var speedScale: Double = 0.5

    // MARK: - Velocity commands (set/cleared by ServoControlPanel)

    /// Joint mode: joint name → direction (±1).  Removing a key stops that joint.
    private(set) var jointCommands: [String: Double] = [:]
    /// Cartesian mode: 6-element [vx, vy, vz, ωx, ωy, ωz]; each component ±1 or 0.
    private(set) var cartesianCommand: [Double] = Array(repeating: 0.0, count: 6)

    // MARK: - Private

    private var timer:    Timer?
    private weak var appState: AppState?

    private static let hz:           Double = 100.0
    private static let dt:           Double = 1.0 / hz
    private static let jointSpeed:   Double = 0.5    // rad/s  at speedScale = 1
    private static let linearSpeed:  Double = 0.08   // m/s   at speedScale = 1
    private static let angularSpeed: Double = 0.5    // rad/s  at speedScale = 1

    // MARK: - Lifecycle

    func start(appState: AppState) {
        guard status == .idle else { return }
        self.appState = appState
        status = .running
        scheduleTimer()
    }

    func stop() {
        timer?.invalidate()
        timer = nil
        clearCommands()
        status = .idle
        appState = nil
    }

    /// Clear the halted state and restart the timer.
    func resume() {
        guard status == .halted, appState != nil else { return }
        status = .running
        scheduleTimer()
    }

    // MARK: - Command API

    func setJoint(_ name: String, direction: Double) {
        if direction == 0 { jointCommands.removeValue(forKey: name) }
        else              { jointCommands[name] = direction }
    }

    func setCartesian(axis: Int, direction: Double) {
        guard (0..<6).contains(axis) else { return }
        cartesianCommand[axis] = direction
    }

    func clearCommands() {
        jointCommands.removeAll()
        cartesianCommand = Array(repeating: 0.0, count: 6)
    }

    // MARK: - Timer

    private func scheduleTimer() {
        let t = Timer(timeInterval: Self.dt, repeats: true) { [weak self] _ in
            Task { @MainActor [weak self] in self?.tick() }
        }
        RunLoop.main.add(t, forMode: .common)
        timer = t
    }

    // MARK: - Tick (100 Hz)

    private func tick() {
        guard status == .running,
              let appState,
              let model = appState.robotModel else { return }

        // Skip tick when nothing is commanded.
        switch mode {
        case .joint:
            guard !jointCommands.isEmpty else { return }
        case .cartesian:
            guard cartesianCommand.contains(where: { $0 != 0 }) else { return }
        }

        let scaledDt = Self.dt * speedScale
        var candidate = appState.jointAngles

        switch mode {
        case .joint:
            for (name, dir) in jointCommands {
                guard let joint = model.joints[name] else { continue }
                let delta = dir * Self.jointSpeed * scaledDt
                let next  = (candidate[name] ?? 0) + delta
                if let lim = joint.limits {
                    candidate[name] = min(max(next, lim.lower), lim.upper)
                } else {
                    candidate[name] = next
                }
            }
        case .cartesian:
            guard let result = cartesianStep(from: candidate, model: model, scaledDt: scaledDt) else {
                return  // IK failed this tick — silently skip
            }
            candidate = result
        }

        // Collision guard — commit only if the candidate is collision-free.
        let fk = ForwardKinematics.compute(model: model, jointAngles: candidate)
        let cr = CollisionChecker.check(
            model:        model,
            fkResult:     fk,
            obstacles:    appState.obstacles,
            disabledPairs: appState.robotSetup.disabledCollisions
        )
        if cr.hasCollision {
            // Halt: stop timer and notify the UI.
            timer?.invalidate()
            timer = nil
            clearCommands()
            status = .halted
        } else {
            appState.jointAngles = candidate
        }
    }

    // MARK: - Cartesian step

    private func cartesianStep(from angles: [String: Double],
                                model: RobotModel,
                                scaledDt: Double) -> [String: Double]? {
        // Resolve EE link: active planning group tip, else last joint's child.
        let tipLink: String
        if let group = appState?.robotSetup.activeGroup {
            tipLink = group.tipLink
        } else {
            guard let lastName = model.orderedActuatedJointNames.last,
                  let joint    = model.joints[lastName] else { return nil }
            tipLink = joint.childLinkName
        }

        let fk = ForwardKinematics.compute(model: model, jointAngles: angles)
        var T  = fk.transform(for: tipLink)

        // Translate in world frame.
        let vx = cartesianCommand[0] * Self.linearSpeed  * scaledDt
        let vy = cartesianCommand[1] * Self.linearSpeed  * scaledDt
        let vz = cartesianCommand[2] * Self.linearSpeed  * scaledDt
        T.columns.3 += SIMD4<Double>(vx, vy, vz, 0)

        // Rotate about world axes (pre-multiply to keep axes world-fixed).
        let rx = cartesianCommand[3] * Self.angularSpeed * scaledDt
        let ry = cartesianCommand[4] * Self.angularSpeed * scaledDt
        let rz = cartesianCommand[5] * Self.angularSpeed * scaledDt
        if rx != 0 { T = simd_double4x4.rotation(angle: rx, axis: SIMD3(1, 0, 0)) * T }
        if ry != 0 { T = simd_double4x4.rotation(angle: ry, axis: SIMD3(0, 1, 0)) * T }
        if rz != 0 { T = simd_double4x4.rotation(angle: rz, axis: SIMD3(0, 0, 1)) * T }

        // Damped-least-squares IK with a small iteration budget for 100 Hz latency.
        let ik = IKSolver.solve(
            model:               model,
            endEffectorLinkName: tipLink,
            targetTransform:     T,
            initialAngles:       angles,
            maxIterations:       6,
            tolerance:           1e-3,
            dampingFactor:       0.05,
            stepScale:           0.5
        )
        return ik.jointAngles
    }
}
