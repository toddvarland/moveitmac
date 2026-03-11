import Foundation
import URDFKit

// MARK: - Planning Group

/// A named kinematic chain used for motion planning and IK.
struct PlanningGroup: Identifiable, Codable, Equatable {
    var id: UUID = UUID()
    var name: String
    /// BFS-ordered joint names that belong to this group.
    var joints: [String]
    /// The tip link for IK (end-effector parent).
    var tipLink: String

    /// Derive joints from the kinematics chain between baseLink and tipLink in model.
    static func joints(from baseLink: String,
                       to tipLink: String,
                       model: RobotModel) -> [String] {
        // Walk parent pointers from tipLink up to baseLink collecting joints.
        // Build child→parentJoint map.
        var childToJoint: [String: Joint] = [:]
        for joint in model.joints.values {
            childToJoint[joint.childLinkName] = joint
        }
        var joints: [String] = []
        var current = tipLink
        while current != baseLink {
            guard let joint = childToJoint[current] else { break }
            if joint.isActuated { joints.append(joint.name) }
            current = joint.parentLinkName
        }
        return joints.reversed()
    }
}

// MARK: - Disabled Collision Pair

/// A link pair explicitly excluded from self-collision checking.
struct DisabledCollisionPair: Codable, Equatable, Hashable, Identifiable {
    var id: String { "\(link1)|\(link2)" }
    var link1: String
    var link2: String
    var reason: DisableReason

    enum DisableReason: String, Codable, CaseIterable {
        case adjacent       = "Adjacent"
        case defaultInCollision = "Default In Collision"
        case neverInCollision   = "Never In Collision"
        case userDisabled   = "User Disabled"
    }

    /// Canonical init — always stores links in sorted order.
    init(_ a: String, _ b: String, reason: DisableReason) {
        if a <= b { link1 = a; link2 = b } else { link1 = b; link2 = a }
        self.reason = reason
    }
}

// MARK: - Robot Setup

/// The full MoveIt-style configuration for a loaded robot.
struct RobotSetup: Codable, Equatable {
    var planningGroups:      [PlanningGroup]       = []
    var disabledCollisions:  [DisabledCollisionPair] = []
    /// Name of the active planning group (used by planner + IK).
    var activePlanningGroup: String?               = nil

    var activeGroup: PlanningGroup? {
        guard let name = activePlanningGroup else { return nil }
        return planningGroups.first { $0.name == name }
    }

    static let empty = RobotSetup()
}

// MARK: - ACM Generator

/// Computes the Allowed Collision Matrix entries by sampling the robot's workspace.
enum ACMGenerator {

    /// Returns disabled pairs for a model. Runs on a background thread.
    static func compute(model: RobotModel,
                        sampleCount: Int = 1_000) -> [DisabledCollisionPair] {
        let joints   = model.orderedActuatedJointNames
        let linkNames = Array(model.links.keys)
        var disabled = Set<DisabledCollisionPair>()

        // 1. Always-adjacent (within 3 hops) → already skipped by CollisionChecker;
        //    mark them explicitly so the UI can show their reason.
        let adjacent = adjacentPairs(model)
        for pair in adjacent {
            disabled.insert(DisabledCollisionPair(pair.linkA, pair.linkB,
                                                  reason: .adjacent))
        }

        // 2. Sample random configs; track which pairs *ever* collide.
        var everCollided = Set<CollisionPair>()
        var neverCollided = Set<CollisionPair>()

        // Default pose: all zeros.
        let defaultAngles = Dictionary(uniqueKeysWithValues: joints.map { ($0, 0.0) })
        let defaultFK = ForwardKinematics.compute(model: model, jointAngles: defaultAngles)
        let defaultResult = CollisionChecker.check(model: model, fkResult: defaultFK,
                                                    obstacles: [])
        for pair in defaultResult.selfCollisions {
            disabled.insert(DisabledCollisionPair(pair.linkA, pair.linkB,
                                                  reason: .defaultInCollision))
        }

        // Random sampling.
        var rng = SystemRandomNumberGenerator()
        let allPairs: [CollisionPair] = {
            var pairs: [CollisionPair] = []
            for i in 0 ..< linkNames.count {
                for j in i+1 ..< linkNames.count {
                    pairs.append(CollisionPair(linkNames[i], linkNames[j]))
                }
            }
            return pairs
        }()

        for pair in allPairs where !disabled.contains(where: {
            ($0.link1 == pair.linkA && $0.link2 == pair.linkB) ||
            ($0.link1 == pair.linkB && $0.link2 == pair.linkA)
        }) {
            neverCollided.insert(pair)
        }

        for _ in 0 ..< sampleCount {
            var angles: [String: Double] = [:]
            for name in joints {
                guard let joint = model.joints[name] else { continue }
                let lo = joint.limits?.lower ?? -Double.pi
                let hi = joint.limits?.upper ??  Double.pi
                angles[name] = Double.random(in: lo...hi, using: &rng)
            }
            let fk = ForwardKinematics.compute(model: model, jointAngles: angles)
            let result = CollisionChecker.check(model: model, fkResult: fk, obstacles: [])
            for pair in result.selfCollisions {
                everCollided.insert(pair)
                neverCollided.remove(pair)
            }
        }

        // 3. Never-in-collision over all samples → disable.
        for pair in neverCollided {
            // Skip if already marked adjacent.
            guard !disabled.contains(where: {
                ($0.link1 == pair.linkA && $0.link2 == pair.linkB) ||
                ($0.link1 == pair.linkB && $0.link2 == pair.linkA)
            }) else { continue }
            disabled.insert(DisabledCollisionPair(pair.linkA, pair.linkB,
                                                  reason: .neverInCollision))
        }

        return Array(disabled).sorted { $0.link1 < $1.link1 }
    }

    // Mirror of CollisionChecker's adjacentPairs for the same hop count.
    private static func adjacentPairs(_ model: RobotModel, hops: Int = 3) -> [CollisionPair] {
        var neighbors: [String: Set<String>] = [:]
        for joint in model.joints.values {
            neighbors[joint.parentLinkName, default: []].insert(joint.childLinkName)
            neighbors[joint.childLinkName,  default: []].insert(joint.parentLinkName)
        }
        var result = Set<CollisionPair>()
        for startLink in model.links.keys {
            var visited = Set<String>([startLink])
            var frontier: Set<String> = [startLink]
            for _ in 0 ..< hops {
                var next = Set<String>()
                for link in frontier {
                    for n in neighbors[link, default: []] where !visited.contains(n) {
                        visited.insert(n); next.insert(n)
                    }
                }
                frontier = next
            }
            visited.remove(startLink)
            for link in visited { result.insert(CollisionPair(startLink, link)) }
        }
        return Array(result)
    }
}
