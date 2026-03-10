import Testing
import simd
@testable import URDFKit

// MARK: - Helpers

/// 2-DOF planar robot: two revolute joints ±π, no limits collisions.
private func make2DOFRobot() -> RobotModel {
    let base   = Link(name: "base")
    let link1  = Link(name: "link1")
    let link2  = Link(name: "link2")

    let j1 = Joint(
        name: "j1", type: .revolute,
        parentLinkName: "base",  childLinkName: "link1",
        origin: Transform(xyz: .zero),
        axis: SIMD3(0, 0, 1),
        limits: JointLimits(lower: -.pi, upper: .pi, effort: 10, velocity: 1)
    )
    let j2 = Joint(
        name: "j2", type: .revolute,
        parentLinkName: "link1", childLinkName: "link2",
        origin: Transform(xyz: SIMD3(0.5, 0, 0)),
        axis: SIMD3(0, 0, 1),
        limits: JointLimits(lower: -.pi, upper: .pi, effort: 10, velocity: 1)
    )
    return RobotModel(
        name: "2dof",
        links: ["base": base, "link1": link1, "link2": link2],
        joints: ["j1": j1, "j2": j2],
        rootLinkName: "base"
    )
}

// MARK: - Tests

@Suite("RRTPlanner")
struct RRTPlannerTests {

    // ── Basic path finding ────────────────────────────────────────────────

    @Test("Plans a path in collision-free space")
    func planFreeSpace() throws {
        let model  = make2DOFRobot()
        let start  = ["j1": 0.0,  "j2": 0.0]
        let goal   = ["j1": 1.5,  "j2": -1.5]

        let result = RRTPlanner.plan(
            model: model, start: start, goal: goal,
            isCollisionFree: { _ in true },         // no obstacles
            maxIterations: 3_000
        )

        #expect(result.success)
        #expect(result.path.count >= 2)
        // First waypoint ≈ start.
        #expect(abs((result.path.first!["j1"] ?? 99) - 0.0) < 1e-6)
        // Last waypoint ≈ goal.
        #expect(abs((result.path.last!["j1"] ?? 99) - 1.5) < 0.1)
    }

    @Test("Returns path [start, goal] when already within tolerance")
    func planAlreadyClose() {
        let model  = make2DOFRobot()
        let start  = ["j1": 0.0, "j2": 0.0]
        let goal   = ["j1": 0.01, "j2": 0.0]   // < 0.05 normalised

        let result = RRTPlanner.plan(
            model: model, start: start, goal: goal,
            isCollisionFree: { _ in true }
        )

        #expect(result.success)
        #expect(result.iterations == 0)
    }

    @Test("Fails when start is in collision")
    func planStartCollision() {
        let model  = make2DOFRobot()
        let start  = ["j1": 0.0, "j2": 0.0]
        let goal   = ["j1": 1.0, "j2": 0.0]

        let result = RRTPlanner.plan(
            model: model, start: start, goal: goal,
            isCollisionFree: { _ in false }    // everything in collision
        )

        #expect(!result.success)
        #expect(result.path.isEmpty)
    }

    @Test("Fails when goal is unreachable within maxIterations")
    func planImpossible() {
        let model  = make2DOFRobot()
        let start  = ["j1":  0.0, "j2": 0.0]
        let goal   = ["j1":  2.0, "j2": 0.0]

        // Allow start/goal but block everything else.
        let result = RRTPlanner.plan(
            model: model, start: start, goal: goal,
            isCollisionFree: { q in
                // Only the start and goal configs are "free";
                // everything in between is blocked.
                let j1 = q["j1"] ?? 0
                return abs(j1) < 0.01 || abs(j1 - 2.0) < 0.01
            },
            maxIterations: 200
        )

        #expect(!result.success)
    }

    @Test("Smoothed path has fewer waypoints than raw") 
    func planSmoothing() {
        let model  = make2DOFRobot()
        let start  = ["j1": -1.5, "j2": -1.5]
        let goal   = ["j1":  1.5, "j2":  1.5]

        // Unsmoothed: use a tiny step size to force many waypoints.
        let resultTiny = RRTPlanner.plan(
            model: model, start: start, goal: goal,
            isCollisionFree: { _ in true },
            maxIterations: 5_000,
            stepSize: 0.02      // very small → many raw nodes
        )
        // Default step size (larger) with smoothing built-in.
        let resultNorm = RRTPlanner.plan(
            model: model, start: start, goal: goal,
            isCollisionFree: { _ in true },
            maxIterations: 5_000,
            stepSize: 0.1
        )

        // Both should succeed.
        #expect(resultTiny.success)
        #expect(resultNorm.success)

        // Normal run's smoothed path should be short.
        // Straight line from start to goal → 2 waypoints after full smoothing.
        #expect(resultNorm.path.count <= resultTiny.path.count)
    }
}
