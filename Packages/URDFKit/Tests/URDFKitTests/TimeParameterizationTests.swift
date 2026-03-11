import Testing
import simd
@testable import URDFKit

// MARK: - Helpers

private func make1DOFRobot(vMax: Double = 1.0, aMax: Double = 2.0) -> RobotModel {
    let base  = Link(name: "base")
    let link1 = Link(name: "link1")
    let joint = Joint(
        name: "j1", type: .revolute,
        parentLinkName: "base", childLinkName: "link1",
        origin: Transform(xyz: .zero),
        axis: SIMD3(0, 0, 1),
        limits: JointLimits(lower: -.pi, upper: .pi, effort: 10, velocity: vMax)
    )
    return RobotModel(
        name: "1dof",
        links: ["base": base, "link1": link1],
        joints: ["j1": joint],
        rootLinkName: "base"
    )
}

// MARK: - Tests

@Suite("TimeParameterization")
struct TimeParameterizationTests {

    @Test("Single waypoint returns zero-duration trajectory")
    func singleWaypoint() {
        let model  = make1DOFRobot()
        let path   = [["j1": 0.0]]
        let timed  = TimeParameterization.apply(path: path, model: model)
        #expect(timed.duration == 0)
        #expect(timed.waypoints.count == 1)
    }

    @Test("Duration is positive for a two-waypoint path")
    func twoWaypointDuration() {
        let model  = make1DOFRobot(vMax: 1.0, aMax: 2.0)
        let path   = [["j1": 0.0], ["j1": 1.0]]
        let timed  = TimeParameterization.apply(path: path, model: model)
        #expect(timed.duration > 0)
        // Trapezoidal: dq=1, v=1, a=2 → dRamp = 1²/2 = 0.5 < 1 → trapezoidal
        // t = 1/1 + 1/2 = 1.5 s
        #expect(abs(timed.duration - 1.5) < 0.01)
    }

    @Test("First and last waypoints have zero velocity")
    func boundarySpeeds() {
        let model  = make1DOFRobot()
        let path   = [["j1": 0.0], ["j1": 0.5], ["j1": 1.0]]
        let timed  = TimeParameterization.apply(path: path, model: model)
        #expect(abs(timed.waypoints.first!.velocities["j1", default: 99]) < 1e-9)
        #expect(abs(timed.waypoints.last!.velocities["j1",  default: 99]) < 1e-9)
    }

    @Test("Monotonically increasing timestamps")
    func monotonicTimes() {
        let model  = make1DOFRobot()
        let path   = (0...5).map { ["j1": Double($0) * 0.2] }
        let timed  = TimeParameterization.apply(path: path, model: model)
        for i in 0 ..< timed.waypoints.count - 1 {
            #expect(timed.waypoints[i + 1].time > timed.waypoints[i].time)
        }
    }

    @Test("Interpolation at t=0 returns start config")
    func interpolateAtZero() {
        let model  = make1DOFRobot()
        let path   = [["j1": 0.0], ["j1": 1.0]]
        let timed  = TimeParameterization.apply(path: path, model: model)
        let q      = timed.angles(at: 0)
        #expect(abs((q["j1"] ?? 99) - 0.0) < 1e-9)
    }

    @Test("Interpolation at t=duration returns goal config")
    func interpolateAtEnd() {
        let model  = make1DOFRobot()
        let path   = [["j1": 0.0], ["j1": 1.0]]
        let timed  = TimeParameterization.apply(path: path, model: model)
        let q      = timed.angles(at: timed.duration)
        #expect(abs((q["j1"] ?? 99) - 1.0) < 1e-6)
    }

    @Test("Interpolation at midpoint is between start and goal")
    func interpolateMidpoint() {
        let model  = make1DOFRobot()
        let path   = [["j1": 0.0], ["j1": 2.0]]
        let timed  = TimeParameterization.apply(path: path, model: model)
        let q      = timed.angles(at: timed.duration / 2)
        let val    = q["j1"] ?? 99
        #expect(val > 0.0 && val < 2.0)
    }

    @Test("Faster joint limit produces shorter duration")
    func fasterLimitShorterDuration() {
        let slow  = TimeParameterization.apply(
            path: [["j1": 0.0], ["j1": 1.0]],
            model: make1DOFRobot(vMax: 0.5)
        )
        let fast  = TimeParameterization.apply(
            path: [["j1": 0.0], ["j1": 1.0]],
            model: make1DOFRobot(vMax: 2.0)
        )
        #expect(fast.duration < slow.duration)
    }
}
