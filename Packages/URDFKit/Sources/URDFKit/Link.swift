import Foundation
import simd

// MARK: - Link

public struct Link {
    public let name: String
    public let visual: Visual?
    public let collision: CollisionGeometry?
    public let inertial: Inertial?

    public init(
        name: String,
        visual: Visual? = nil,
        collision: CollisionGeometry? = nil,
        inertial: Inertial? = nil
    ) {
        self.name = name
        self.visual = visual
        self.collision = collision
        self.inertial = inertial
    }
}

// MARK: - Visual / Collision

public struct Visual {
    public let origin: Transform
    public let geometry: Geometry
    public let materialName: String?

    public init(origin: Transform, geometry: Geometry, materialName: String? = nil) {
        self.origin = origin
        self.geometry = geometry
        self.materialName = materialName
    }
}

public struct CollisionGeometry {
    public let origin: Transform
    public let geometry: Geometry

    public init(origin: Transform, geometry: Geometry) {
        self.origin = origin
        self.geometry = geometry
    }
}

// MARK: - Geometry

public enum Geometry {
    case mesh(filename: String, scale: SIMD3<Double>)
    case box(size: SIMD3<Double>)
    case cylinder(radius: Double, length: Double)
    case sphere(radius: Double)
}

// MARK: - Inertial

public struct Inertial {
    public let origin: Transform
    public let mass: Double
    public let ixx, ixy, ixz, iyy, iyz, izz: Double

    public init(
        origin: Transform,
        mass: Double,
        ixx: Double, ixy: Double, ixz: Double,
        iyy: Double, iyz: Double, izz: Double
    ) {
        self.origin = origin
        self.mass = mass
        self.ixx = ixx; self.ixy = ixy; self.ixz = ixz
        self.iyy = iyy; self.iyz = iyz; self.izz = izz
    }
}

// MARK: - Transform

/// A pose expressed as translation (xyz, metres) and rotation (rpy, radians).
public struct Transform {
    public let xyz: SIMD3<Double>
    /// Roll-pitch-yaw (extrinsic XYZ) rotation in radians.
    public let rpy: SIMD3<Double>

    public static let identity = Transform(xyz: .zero, rpy: .zero)

    public init(xyz: SIMD3<Double> = .zero, rpy: SIMD3<Double> = .zero) {
        self.xyz = xyz
        self.rpy = rpy
    }
}
