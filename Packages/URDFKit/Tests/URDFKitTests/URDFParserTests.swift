import Testing
import Foundation
@testable import URDFKit

// MARK: - Inline URDF fixtures

private let simpleURDF = """
<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.2 0.3"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.2 0.3"/>
      </geometry>
    </collision>
  </link>
  <link name="link_1">
    <visual>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </visual>
  </link>
  <link name="link_2">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.07"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="100" velocity="2.0"/>
  </joint>
  <joint name="joint_2" type="prismatic">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="0.5"/>
  </joint>
</robot>
"""

private let fixedJointURDF = """
<?xml version="1.0"?>
<robot name="fixed_robot">
  <link name="base"/>
  <link name="tool"/>
  <joint name="fixed_joint" type="fixed">
    <parent link="base"/>
    <child link="tool"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>
</robot>
"""

// MARK: - Test Suite

@Suite("URDF Parser")
struct URDFParserTests {

    // MARK: Robot-level

    @Test("Parses robot name")
    func robotName() throws {
        let model = try parse(simpleURDF)
        #expect(model.name == "test_robot")
    }

    @Test("Parses link count")
    func linkCount() throws {
        let model = try parse(simpleURDF)
        #expect(model.links.count == 3)
    }

    @Test("Parses joint count")
    func jointCount() throws {
        let model = try parse(simpleURDF)
        #expect(model.joints.count == 2)
    }

    @Test("Root link is base_link")
    func rootLink() throws {
        let model = try parse(simpleURDF)
        #expect(model.rootLinkName == "base_link")
    }

    // MARK: Joints

    @Test("Revolute joint type parsed")
    func revoluteJointType() throws {
        let model = try parse(simpleURDF)
        let joint = try #require(model.joints["joint_1"])
        #expect(joint.type == .revolute)
    }

    @Test("Prismatic joint type parsed")
    func prismaticJointType() throws {
        let model = try parse(simpleURDF)
        let joint = try #require(model.joints["joint_2"])
        #expect(joint.type == .prismatic)
    }

    @Test("Fixed joint is not actuated")
    func fixedJointNotActuated() throws {
        let model = try parse(fixedJointURDF)
        let joint = try #require(model.joints["fixed_joint"])
        #expect(joint.isActuated == false)
    }

    @Test("Joint parent and child links")
    func jointParentChild() throws {
        let model = try parse(simpleURDF)
        let joint = try #require(model.joints["joint_1"])
        #expect(joint.parentLinkName == "base_link")
        #expect(joint.childLinkName == "link_1")
    }

    @Test("Joint origin xyz parsed")
    func jointOriginXYZ() throws {
        let model = try parse(simpleURDF)
        let joint = try #require(model.joints["joint_1"])
        #expect(abs(joint.origin.xyz.z - 0.1) < 1e-9)
    }

    @Test("Joint axis parsed")
    func jointAxis() throws {
        let model = try parse(simpleURDF)
        let j2 = try #require(model.joints["joint_2"])
        #expect(j2.axis.y == 1.0)
    }

    // MARK: Joint Limits

    @Test("Revolute joint limits lower/upper")
    func jointLimitsRevoluete() throws {
        let model = try parse(simpleURDF)
        let joint = try #require(model.joints["joint_1"])
        let lim = try #require(joint.limits)
        #expect(abs(lim.lower - (-3.14159)) < 1e-4)
        #expect(abs(lim.upper -   3.14159)  < 1e-4)
    }

    @Test("Joint limits effort and velocity")
    func jointLimitsEffortVelocity() throws {
        let model = try parse(simpleURDF)
        let joint = try #require(model.joints["joint_1"])
        let lim = try #require(joint.limits)
        #expect(lim.effort == 100)
        #expect(lim.velocity == 2.0)
    }

    // MARK: Geometry

    @Test("Box geometry size parsed")
    func boxGeometry() throws {
        let model = try parse(simpleURDF)
        let link = try #require(model.links["base_link"])
        let visual = try #require(link.visual)
        guard case .box(let size) = visual.geometry else {
            Issue.record("Expected box geometry, got \(visual.geometry)")
            return
        }
        #expect(size.x == 0.1)
        #expect(size.y == 0.2)
        #expect(size.z == 0.3)
    }

    @Test("Cylinder geometry parsed")
    func cylinderGeometry() throws {
        let model = try parse(simpleURDF)
        let link = try #require(model.links["link_1"])
        let visual = try #require(link.visual)
        guard case .cylinder(let r, let l) = visual.geometry else {
            Issue.record("Expected cylinder geometry")
            return
        }
        #expect(r == 0.05)
        #expect(l == 0.5)
    }

    @Test("Sphere geometry parsed")
    func sphereGeometry() throws {
        let model = try parse(simpleURDF)
        let link = try #require(model.links["link_2"])
        let visual = try #require(link.visual)
        guard case .sphere(let r) = visual.geometry else {
            Issue.record("Expected sphere geometry")
            return
        }
        #expect(r == 0.07)
    }

    @Test("Collision geometry parsed independently of visual")
    func collisionGeometry() throws {
        let model = try parse(simpleURDF)
        let link = try #require(model.links["base_link"])
        let col = try #require(link.collision)
        if case .box = col.geometry { } else {
            Issue.record("Expected box collision geometry")
        }
    }

    // MARK: Ordered Joints

    @Test("Ordered actuated joint names from root")
    func orderedActuatedJoints() throws {
        let model = try parse(simpleURDF)
        let names = model.orderedActuatedJointNames
        // joint_1 connects base_link→link_1, joint_2 connects link_1→link_2
        #expect(names == ["joint_1", "joint_2"])
    }

    @Test("Fixed joint excluded from actuated list")
    func fixedJointExcluded() throws {
        let model = try parse(fixedJointURDF)
        #expect(model.orderedActuatedJointNames.isEmpty)
    }

    // MARK: Error cases

    @Test("Throws for missing file")
    func missingFile() {
        let url = URL(fileURLWithPath: "/tmp/does_not_exist_\(UUID().uuidString).urdf")
        #expect(throws: URDFError.self) {
            try URDFParser.parse(contentsOf: url)
        }
    }

    @Test("Throws for unknown joint type")
    func unknownJointType() {
        let bad = """
        <?xml version="1.0"?>
        <robot name="bad">
          <link name="a"/><link name="b"/>
          <joint name="j" type="teleport">
            <parent link="a"/><child link="b"/>
          </joint>
        </robot>
        """
        #expect(throws: URDFError.self) {
            try parse(bad)
        }
    }
}

// MARK: - Test helpers

private func parse(_ urdf: String) throws -> RobotModel {
    let tmp = FileManager.default.temporaryDirectory
        .appendingPathComponent(UUID().uuidString + ".urdf")
    try urdf.write(to: tmp, atomically: true, encoding: .utf8)
    defer { try? FileManager.default.removeItem(at: tmp) }
    return try URDFParser.parse(contentsOf: tmp)
}
