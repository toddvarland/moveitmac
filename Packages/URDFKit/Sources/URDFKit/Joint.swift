import Foundation
import simd

// MARK: - Joint Type

public enum JointType: String {
    case revolute
    case continuous
    case prismatic
    case fixed
    case floating
    case planar
}

// MARK: - Joint

public struct Joint {
    public let name: String
    public let type: JointType
    public let parentLinkName: String
    public let childLinkName: String
    public let origin: Transform
    /// Axis of rotation or translation in the joint frame.
    public let axis: SIMD3<Double>
    public let limits: JointLimits?

    public init(
        name: String,
        type: JointType,
        parentLinkName: String,
        childLinkName: String,
        origin: Transform,
        axis: SIMD3<Double> = SIMD3(0, 0, 1),
        limits: JointLimits? = nil
    ) {
        self.name = name
        self.type = type
        self.parentLinkName = parentLinkName
        self.childLinkName = childLinkName
        self.origin = origin
        self.axis = axis
        self.limits = limits
    }

    /// True for joints that have a configurable position (revolute, continuous, prismatic).
    public var isActuated: Bool {
        type == .revolute || type == .continuous || type == .prismatic
    }
}

// MARK: - Joint Limits

public struct JointLimits {
    /// Lower position limit (radians for revolute/continuous, metres for prismatic).
    public let lower: Double
    /// Upper position limit.
    public let upper: Double
    /// Maximum effort (N·m or N).
    public let effort: Double
    /// Maximum velocity (rad/s or m/s).
    public let velocity: Double

    public init(lower: Double, upper: Double, effort: Double, velocity: Double) {
        self.lower = lower
        self.upper = upper
        self.effort = effort
        self.velocity = velocity
    }
}
