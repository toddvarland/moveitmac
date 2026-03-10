import Foundation
import simd

/// The complete kinematic description of a robot, parsed from URDF + SRDF.
public struct RobotModel {
    public let name: String
    public let links: [String: Link]
    public let joints: [String: Joint]
    public let rootLinkName: String

    public init(
        name: String,
        links: [String: Link],
        joints: [String: Joint],
        rootLinkName: String
    ) {
        self.name = name
        self.links = links
        self.joints = joints
        self.rootLinkName = rootLinkName
    }

    // MARK: - Convenience

    /// Actuated joints (revolute, continuous, prismatic) in BFS order from root.
    public var orderedActuatedJointNames: [String] {
        orderedJointNames.filter { joints[$0]?.isActuated == true }
    }

    /// All joints in BFS order from the root link.
    public var orderedJointNames: [String] {
        var result: [String] = []
        var queue = [rootLinkName]
        while !queue.isEmpty {
            let current = queue.removeFirst()
            let children = joints.values
                .filter { $0.parentLinkName == current }
                .sorted { $0.name < $1.name }
            for joint in children {
                result.append(joint.name)
                queue.append(joint.childLinkName)
            }
        }
        return result
    }
}
