import Testing
import Foundation
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
