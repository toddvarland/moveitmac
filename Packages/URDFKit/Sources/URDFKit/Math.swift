import simd

// MARK: - 4×4 homogeneous transform as simd_double4x4

extension simd_double4x4 {
    /// Identity matrix.
    public static let identity = matrix_identity_double4x4

    /// Construct from translation + RPY (extrinsic XYZ: roll about X, pitch about Y, yaw about Z).
    public static func from(transform: Transform) -> simd_double4x4 {
        let t = translation(transform.xyz)
        let r = rotation(rpy: transform.rpy)
        return t * r
    }

    /// Pure translation.
    public static func translation(_ v: SIMD3<Double>) -> simd_double4x4 {
        var m = matrix_identity_double4x4
        m.columns.3 = SIMD4(v.x, v.y, v.z, 1)
        return m
    }

    /// Rotation from roll-pitch-yaw (extrinsic XYZ convention, as URDF specifies).
    public static func rotation(rpy: SIMD3<Double>) -> simd_double4x4 {
        let (cr, sr) = (cos(rpy.x), sin(rpy.x))   // roll  (X)
        let (cp, sp) = (cos(rpy.y), sin(rpy.y))   // pitch (Y)
        let (cy, sy) = (cos(rpy.z), sin(rpy.z))   // yaw   (Z)

        // R = Rz(yaw) * Ry(pitch) * Rx(roll)  — standard URDF extrinsic XYZ
        let r00 = cy * cp;  let r01 = cy * sp * sr - sy * cr;  let r02 = cy * sp * cr + sy * sr
        let r10 = sy * cp;  let r11 = sy * sp * sr + cy * cr;  let r12 = sy * sp * cr - cy * sr
        let r20 = -sp;      let r21 = cp * sr;                  let r22 = cp * cr

        return simd_double4x4(columns: (
            SIMD4(r00, r10, r20, 0),
            SIMD4(r01, r11, r21, 0),
            SIMD4(r02, r12, r22, 0),
            SIMD4(  0,   0,   0, 1)
        ))
    }

    /// Rotation by `angle` radians about an arbitrary unit `axis`.
    public static func rotation(angle: Double, axis: SIMD3<Double>) -> simd_double4x4 {
        let u = normalize(axis)
        let c = cos(angle), s = sin(angle), t = 1 - cos(angle)
        let (x, y, z) = (u.x, u.y, u.z)

        return simd_double4x4(columns: (
            SIMD4(t*x*x + c,   t*x*y + s*z, t*x*z - s*y, 0),
            SIMD4(t*x*y - s*z, t*y*y + c,   t*y*z + s*x, 0),
            SIMD4(t*x*z + s*y, t*y*z - s*x, t*z*z + c,   0),
            SIMD4(          0,           0,           0,  1)
        ))
    }

    /// Extract the translation vector from column 3.
    public var translation: SIMD3<Double> {
        SIMD3(columns.3.x, columns.3.y, columns.3.z)
    }
}
