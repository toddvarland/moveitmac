import simd

/// Forward kinematics result: the world-frame transform of every link.
public struct FKResult {
    /// World-frame 4×4 homogeneous transform for each link, keyed by link name.
    public let linkTransforms: [String: simd_double4x4]

    /// World-frame transform for a specific link (identity if unknown).
    public func transform(for linkName: String) -> simd_double4x4 {
        linkTransforms[linkName] ?? .identity
    }

    /// World-frame position of a link's origin.
    public func position(of linkName: String) -> SIMD3<Double> {
        transform(for: linkName).translation
    }
}

// MARK: - ForwardKinematics

/// Pure-Swift product-of-exponentials / URDF-convention FK engine.
///
/// Thread-safe: `RobotModel` and `FKResult` are value types; `compute` has no
/// side-effects and can be called from any queue.
public enum ForwardKinematics {

    /// Compute the world-frame transform of every link given a joint configuration.
    ///
    /// - Parameters:
    ///   - model: The robot's kinematic model.
    ///   - jointAngles: Map from joint name → position (radians for revolute/continuous,
    ///                  metres for prismatic). Missing joints default to 0.
    /// - Returns: An `FKResult` containing the world frame transform of every link.
    public static func compute(
        model: RobotModel,
        jointAngles: [String: Double]
    ) -> FKResult {

        // Map joint → angle (missing = 0)
        func q(_ name: String) -> Double { jointAngles[name] ?? 0.0 }

        // BFS from root link, accumulating world-frame transforms.
        // worldT[linkName] = transform from world to that link's origin frame.
        var worldT: [String: simd_double4x4] = [:]
        worldT[model.rootLinkName] = .identity

        var queue = [model.rootLinkName]
        while !queue.isEmpty {
            let parentLinkName = queue.removeFirst()
            let parentWorld = worldT[parentLinkName] ?? .identity

            // Find all joints whose parent is this link
            let childJoints = model.joints.values
                .filter { $0.parentLinkName == parentLinkName }

            for joint in childJoints {
                // T_joint_origin: fixed offset from parent link origin to joint frame
                let T_origin = simd_double4x4.from(transform: joint.origin)

                // T_joint_motion: the actuated motion about/along the joint axis
                let T_motion: simd_double4x4
                switch joint.type {
                case .revolute, .continuous:
                    T_motion = .rotation(angle: q(joint.name), axis: joint.axis)
                case .prismatic:
                    T_motion = .translation(joint.axis * q(joint.name))
                case .fixed, .floating, .planar:
                    T_motion = .identity
                }

                // Child link origin in world frame
                let childWorld = parentWorld * T_origin * T_motion
                worldT[joint.childLinkName] = childWorld
                queue.append(joint.childLinkName)
            }
        }

        return FKResult(linkTransforms: worldT)
    }
}
