import Testing
import Foundation
import simd
import URDFKit
@testable import MoveItMac

@Suite("AppState")
struct AppStateTests {

    @Test("Initial state has no robot loaded")
    @MainActor
    func initialStateEmpty() {
        let state = AppState()
        #expect(state.isRobotLoaded == false)
        #expect(state.robotName == nil)
        #expect(state.jointAngles.isEmpty)
        #expect(state.obstacles.isEmpty)
    }

    @Test("Ordered joint names is empty before load")
    @MainActor
    func orderedJointsBeforeLoad() {
        let state = AppState()
        #expect(state.orderedJointNames.isEmpty)
    }
}

@Suite("CollisionChecker")
struct CollisionCheckerTests {

    @Test("Panda midpoint pose is collision-free")
    func pandaMidpointPoseNoCollision() throws {
        let urdfURL = URL(fileURLWithPath: "/Users/toddvarland/Documents/GitHub/moveit-clone/Resources/panda.urdf")
        let model = try URDFParser.parse(contentsOf: urdfURL)
        var angles: [String: Double] = [:]
        for joint in model.joints.values where joint.isActuated {
            if let lim = joint.limits { angles[joint.name] = (lim.lower + lim.upper) / 2 }
        }
        let fk = ForwardKinematics.compute(model: model, jointAngles: angles)
        let result = CollisionChecker.check(model: model, fkResult: fk, obstacles: [])
        #expect(!result.hasCollision, "Unexpected self-collision at midpoint pose: \(result.selfCollisions)")
    }
}

@Suite("IKSolver")
struct IKSolverTests {

    private static let pandaURL = URL(
        fileURLWithPath: "/Users/toddvarland/Documents/GitHub/moveit-clone/Resources/panda.urdf"
    )

    /// Load the Panda model and return a midpoint starting configuration.
    private static func pandaSetup() throws -> (RobotModel, [String: Double]) {
        let model = try URDFParser.parse(contentsOf: pandaURL)
        var angles: [String: Double] = [:]
        for joint in model.joints.values where joint.isActuated {
            if let lim = joint.limits { angles[joint.name] = (lim.lower + lim.upper) / 2 }
        }
        return (model, angles)
    }

    // MARK: - Round-trip: FK → IK → FK should recover the original pose

    @Test("IK round-trip: FK(IK(target)) ≈ target position")
    func roundTripPosition() throws {
        let (model, midAngles) = try Self.pandaSetup()
        let eeName = "panda_link8"

        // Step 1: Pick a reachable target by FK-ing a known configuration.
        var targetAngles = midAngles
        targetAngles["panda_joint1"] = 0.3
        targetAngles["panda_joint2"] = -0.5
        targetAngles["panda_joint4"] = -1.8
        let targetFk = ForwardKinematics.compute(model: model, jointAngles: targetAngles)
        let targetTransform = targetFk.transform(for: eeName)

        // Step 2: Solve IK from the midpoint start.
        let result = IKSolver.solve(
            model: model,
            endEffectorLinkName: eeName,
            targetTransform: targetTransform,
            initialAngles: midAngles
        )

        // Step 3: FK the solution and check position error ≤ 1 mm.
        let solvedFk = ForwardKinematics.compute(model: model, jointAngles: result.jointAngles)
        let solvedPos = solvedFk.transform(for: eeName).translation
        let targetPos = targetTransform.translation
        let posErr = simd_length(solvedPos - targetPos)

        #expect(result.converged, "IK did not converge (finalError=\(result.finalError))")
        #expect(posErr < 1e-3, "Position error \(posErr) m exceeds 1 mm threshold")
    }

    @Test("IK converges from exact seed")
    func convergenceFromSeed() throws {
        let (model, midAngles) = try Self.pandaSetup()
        let eeName = "panda_link8"
        let fk = ForwardKinematics.compute(model: model, jointAngles: midAngles)
        let target = fk.transform(for: eeName)

        // Seed is the exact solution — should converge in very few iterations.
        let result = IKSolver.solve(
            model: model,
            endEffectorLinkName: eeName,
            targetTransform: target,
            initialAngles: midAngles,
            tolerance: 1e-5
        )
        #expect(result.converged)
        #expect(result.iterations < 20)
    }

    @Test("IK respects joint limits")
    func jointLimitsRespected() throws {
        let (model, midAngles) = try Self.pandaSetup()
        let eeName = "panda_link8"
        let fk = ForwardKinematics.compute(model: model, jointAngles: midAngles)
        let target = fk.transform(for: eeName)

        let result = IKSolver.solve(
            model: model,
            endEffectorLinkName: eeName,
            targetTransform: target,
            initialAngles: midAngles
        )

        for joint in model.joints.values where joint.isActuated {
            if let lim = joint.limits, let angle = result.jointAngles[joint.name] {
                #expect(angle >= lim.lower - 1e-9 && angle <= lim.upper + 1e-9,
                        "Joint \(joint.name) angle \(angle) violates limits [\(lim.lower), \(lim.upper)]")
            }
        }
    }
}
