import Testing
import simd
@testable import URDFKit

// MARK: - Helpers

private let pi = Double.pi

/// Build a minimal single-joint revolute robot:
///   base_link → joint_z (revolute, Z-axis, offset 0.5 m up) → link_1
private func makeRevoluteZRobot() -> RobotModel {
    let base = Link(name: "base_link")
    let link1 = Link(name: "link_1")
    let joint = Joint(
        name: "joint_z",
        type: .revolute,
        parentLinkName: "base_link",
        childLinkName: "link_1",
        origin: Transform(xyz: SIMD3(0, 0, 0.5)),
        axis: SIMD3(0, 0, 1),
        limits: JointLimits(lower: -pi, upper: pi, effort: 100, velocity: 1)
    )
    return RobotModel(
        name: "test", links: ["base_link": base, "link_1": link1],
        joints: ["joint_z": joint], rootLinkName: "base_link"
    )
}

/// Build a two-joint chain:
///   base → joint_y (revolute, Y, +1 m X) → link_1
///         joint_y2 (revolute, Y, +1 m X from link_1) → link_2
private func makeTwoJointRobot() -> RobotModel {
    let base  = Link(name: "base_link")
    let link1 = Link(name: "link_1")
    let link2 = Link(name: "link_2")
    let j1 = Joint(
        name: "j1", type: .revolute,
        parentLinkName: "base_link", childLinkName: "link_1",
        origin: Transform(xyz: SIMD3(1, 0, 0)), axis: SIMD3(0, 1, 0)
    )
    let j2 = Joint(
        name: "j2", type: .revolute,
        parentLinkName: "link_1", childLinkName: "link_2",
        origin: Transform(xyz: SIMD3(1, 0, 0)), axis: SIMD3(0, 1, 0)
    )
    return RobotModel(
        name: "two_joint",
        links: ["base_link": base, "link_1": link1, "link_2": link2],
        joints: ["j1": j1, "j2": j2],
        rootLinkName: "base_link"
    )
}

/// Construct a prismatic-joint robot:
///   base → prismatic_x (X-axis) → slider
private func makePrismaticRobot() -> RobotModel {
    let base   = Link(name: "base_link")
    let slider = Link(name: "slider")
    let joint  = Joint(
        name: "slide_x", type: .prismatic,
        parentLinkName: "base_link", childLinkName: "slider",
        origin: Transform(xyz: SIMD3(0, 0, 0)), axis: SIMD3(1, 0, 0),
        limits: JointLimits(lower: -0.5, upper: 0.5, effort: 50, velocity: 0.5)
    )
    return RobotModel(
        name: "prismatic",
        links: ["base_link": base, "slider": slider],
        joints: ["slide_x": joint],
        rootLinkName: "base_link"
    )
}

// ─── close enough comparison ─────────────────────────────────────────────────
private func almostEqual(_ a: Double, _ b: Double, tol: Double = 1e-9) -> Bool {
    abs(a - b) < tol
}
private func almostEqual(_ a: SIMD3<Double>, _ b: SIMD3<Double>, tol: Double = 1e-9) -> Bool {
    almostEqual(a.x, b.x, tol: tol) && almostEqual(a.y, b.y, tol: tol) && almostEqual(a.z, b.z, tol: tol)
}

// MARK: - Test Suite

@Suite("Forward Kinematics")
struct ForwardKinematicsTests {

    // MARK: Root link

    @Test("Root link is always at identity")
    func rootAtIdentity() {
        let model = makeRevoluteZRobot()
        let fk = ForwardKinematics.compute(model: model, jointAngles: [:])
        let t = fk.transform(for: "base_link")
        let ref = simd_double4x4.identity
        for col in 0..<4 {
            for row in 0..<4 {
                #expect(almostEqual(t[col][row], ref[col][row]))
            }
        }
    }

    // MARK: Revolute joint — zero angle

    @Test("Zero-angle revolute: child at joint origin offset")
    func revoluteZeroAngle() {
        let model = makeRevoluteZRobot()
        let fk = ForwardKinematics.compute(model: model, jointAngles: ["joint_z": 0])
        let pos = fk.position(of: "link_1")
        #expect(almostEqual(pos, SIMD3(0, 0, 0.5)))
    }

    // MARK: Revolute joint — 90 degrees

    @Test("90° revolute about Z: child local X-axis aligns with world Y")
    func revolute90DegZ() {
        // A revolute joint does NOT move the child link origin — it only reorients
        // the child frame. Position stays at the joint origin; the orientation
        // changes. We verify the child's local X-axis (column 0 of the world
        // transform) now points in world +Y after a 90° Z rotation.
        let base = Link(name: "base_link")
        let link1 = Link(name: "link_1")
        let joint = Joint(
            name: "j", type: .revolute,
            parentLinkName: "base_link", childLinkName: "link_1",
            origin: Transform(xyz: SIMD3(1, 0, 0)), axis: SIMD3(0, 0, 1)
        )
        let m = RobotModel(
            name: "r", links: ["base_link": base, "link_1": link1],
            joints: ["j": joint], rootLinkName: "base_link"
        )
        let fk = ForwardKinematics.compute(model: m, jointAngles: ["j": pi / 2])
        // Position stays at joint origin (1,0,0)
        let pos = fk.position(of: "link_1")
        #expect(almostEqual(pos, SIMD3(1, 0, 0), tol: 1e-9))
        // Child local X column should now point in world +Y
        let t = fk.transform(for: "link_1")
        let localX = SIMD3(t.columns.0.x, t.columns.0.y, t.columns.0.z)
        #expect(almostEqual(localX, SIMD3(0, 1, 0), tol: 1e-9))
    }

    @Test("180° revolute about Z: child local X-axis points world -X")
    func revolute180DegZ() {
        // Same as above: position stays at joint origin; orientation flips.
        let base = Link(name: "base_link")
        let link1 = Link(name: "link_1")
        let joint = Joint(
            name: "j", type: .revolute,
            parentLinkName: "base_link", childLinkName: "link_1",
            origin: Transform(xyz: SIMD3(1, 0, 0)), axis: SIMD3(0, 0, 1)
        )
        let m = RobotModel(
            name: "r", links: ["base_link": base, "link_1": link1],
            joints: ["j": joint], rootLinkName: "base_link"
        )
        let fk = ForwardKinematics.compute(model: m, jointAngles: ["j": pi])
        // Position stays at (1,0,0)
        let pos = fk.position(of: "link_1")
        #expect(almostEqual(pos, SIMD3(1, 0, 0), tol: 1e-9))
        // After 180° about Z the local X column should point in world -X
        let t = fk.transform(for: "link_1")
        let localX = SIMD3(t.columns.0.x, t.columns.0.y, t.columns.0.z)
        #expect(almostEqual(localX, SIMD3(-1, 0, 0), tol: 1e-9))
    }

    // MARK: Prismatic joint

    @Test("Prismatic at zero: child at origin")
    func prismaticZero() {
        let model = makePrismaticRobot()
        let fk = ForwardKinematics.compute(model: model, jointAngles: ["slide_x": 0])
        let pos = fk.position(of: "slider")
        #expect(almostEqual(pos, SIMD3(0, 0, 0)))
    }

    @Test("Prismatic at 0.3 m along X")
    func prismatic03m() {
        let model = makePrismaticRobot()
        let fk = ForwardKinematics.compute(model: model, jointAngles: ["slide_x": 0.3])
        let pos = fk.position(of: "slider")
        #expect(almostEqual(pos, SIMD3(0.3, 0, 0)))
    }

    @Test("Prismatic negative displacement")
    func prismaticNegative() {
        let model = makePrismaticRobot()
        let fk = ForwardKinematics.compute(model: model, jointAngles: ["slide_x": -0.25])
        let pos = fk.position(of: "slider")
        #expect(almostEqual(pos, SIMD3(-0.25, 0, 0)))
    }

    // MARK: Fixed joint

    @Test("Fixed joint: child at constant offset regardless of any other joint")
    func fixedJoint() {
        let base  = Link(name: "base_link")
        let tool  = Link(name: "tool")
        let joint = Joint(
            name: "fixed_j", type: .fixed,
            parentLinkName: "base_link", childLinkName: "tool",
            origin: Transform(xyz: SIMD3(0, 0, 0.25))
        )
        let m = RobotModel(
            name: "fixed_robot",
            links: ["base_link": base, "tool": tool],
            joints: ["fixed_j": joint],
            rootLinkName: "base_link"
        )
        let fk = ForwardKinematics.compute(model: m, jointAngles: [:])
        let pos = fk.position(of: "tool")
        #expect(almostEqual(pos, SIMD3(0, 0, 0.25)))
    }

    // MARK: Two-joint chain (pose composition)

    @Test("Two-joint chain at zero: link_2 at (2, 0, 0)")
    func twoJointZero() {
        let model = makeTwoJointRobot()
        let fk = ForwardKinematics.compute(model: model, jointAngles: ["j1": 0, "j2": 0])
        let pos = fk.position(of: "link_2")
        #expect(almostEqual(pos, SIMD3(2, 0, 0)))
    }

    @Test("Two-joint chain: j1=90° about Y rotates link_1 frame; link_2 offset into -Z")
    func twoJointJ1_90() {
        // j1: revolute Y at (1,0,0). Position of link_1 stays at (1,0,0).
        // j2: revolute Y at (1,0,0) in link_1 frame. At j2=0 the offset (1,0,0)
        // is expressed in link_1's rotated frame. After j1=90° about Y,
        // link_1's local X points in world -Z, so link_2 = (1,0,0) + (0,0,-1) = (1,0,-1).
        let model = makeTwoJointRobot()
        let fk = ForwardKinematics.compute(model: model, jointAngles: ["j1": pi / 2, "j2": 0])
        let p1 = fk.position(of: "link_1")
        #expect(almostEqual(p1, SIMD3(1, 0, 0), tol: 1e-9))
        let p2 = fk.position(of: "link_2")
        #expect(almostEqual(p2, SIMD3(1, 0, -1), tol: 1e-9))
    }

    @Test("Two-joint chain: j1=90°, j2=-90° — link_2 still at (1,0,-1)")
    func twoJointJ1_90_J2_neg90() {
        // Position of link_2 is determined by the offset from link_1, which is (1,0,0)
        // expressed in link_1's rotated frame. j2 only changes link_2's orientation,
        // not its origin position. So link_2 stays at (1,0,-1).
        let model = makeTwoJointRobot()
        let fk = ForwardKinematics.compute(model: model, jointAngles: ["j1": pi / 2, "j2": -pi / 2])
        let p2 = fk.position(of: "link_2")
        #expect(almostEqual(p2, SIMD3(1, 0, -1), tol: 1e-9))
    }

    // MARK: Missing joint defaults to zero

    @Test("Missing joint angle defaults to 0")
    func missingJointDefaultsToZero() {
        let model = makeRevoluteZRobot()
        let fk1 = ForwardKinematics.compute(model: model, jointAngles: [:])
        let fk2 = ForwardKinematics.compute(model: model, jointAngles: ["joint_z": 0])
        let p1 = fk1.position(of: "link_1")
        let p2 = fk2.position(of: "link_1")
        #expect(almostEqual(p1, p2))
    }

    // MARK: Math helpers

    @Test("simd_double4x4 rotation 90° about Z produces correct matrix")
    func rotationMatrix90Z() {
        let m = simd_double4x4.rotation(angle: pi / 2, axis: SIMD3(0, 0, 1))
        // Column 0 should be (0, 1, 0, 0) and column 1 should be (-1, 0, 0, 0)
        #expect(almostEqual(m.columns.0.x,  0, tol: 1e-9))
        #expect(almostEqual(m.columns.0.y,  1, tol: 1e-9))
        #expect(almostEqual(m.columns.1.x, -1, tol: 1e-9))
        #expect(almostEqual(m.columns.1.y,  0, tol: 1e-9))
    }

    @Test("simd_double4x4 translation vector is recoverable")
    func translationVector() {
        let m = simd_double4x4.translation(SIMD3(3, 4, 5))
        #expect(m.translation == SIMD3(3, 4, 5))
    }

    @Test("RPY rotation identity produces identity matrix")
    func rpyIdentity() {
        let m = simd_double4x4.rotation(rpy: .zero)
        let id = simd_double4x4.identity
        for col in 0..<4 {
            for row in 0..<4 {
                #expect(almostEqual(m[col][row], id[col][row]))
            }
        }
    }
}
