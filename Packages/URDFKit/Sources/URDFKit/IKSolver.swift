import simd
import Foundation

// MARK: - IK Result

/// The outcome of a single IK solve attempt.
public struct IKResult {
    /// Joint angles that best satisfy the target, keyed by joint name.
    public let jointAngles: [String: Double]
    /// True if the solver converged within `tolerance`.
    public let converged: Bool
    /// Number of iterations taken.
    public let iterations: Int
    /// Magnitude of the Cartesian error vector at termination (metres / radians).
    public let finalError: Double
}

// MARK: - IK Solver

/// Numerical Jacobian pseudo-inverse IK solver (damped least squares).
///
/// Algorithm (per iteration):
///   1. FK → end-effector world transform T_ee
///   2. Compute 6×n Jacobian numerically via finite differences (δq = 1e-6 rad)
///   3. Error = [Δx, Δy, Δz, Δroll, Δpitch, Δyaw] as a 6-vector
///   4. dq = Jᵀ (J Jᵀ + λI)⁻¹ · err   (damped pseudo-inverse, λ = 0.01)
///   5. Clamp updated angles to joint limits
///   6. Repeat until |err| < tolerance or maxIterations
///
/// Thread-safe: all inputs are value types; no side effects.
public enum IKSolver {

    // MARK: - Public API

    /// Solve for joint angles that place `endEffectorLinkName` at `targetTransform`.
    ///
    /// - Parameters:
    ///   - model: The robot's kinematic model.
    ///   - endEffectorLinkName: Name of the link to position (e.g. `"panda_link8"`).
    ///   - targetTransform: Desired world-frame pose as a 4×4 homogeneous matrix.
    ///   - initialAngles: Starting joint configuration (angles in radians / metres).
    ///   - maxIterations: Maximum Newton-Raphson iterations before giving up.
    ///   - tolerance: Convergence threshold on the 6-DOF error vector magnitude.
    ///   - dampingFactor: λ in the damped pseudo-inverse (higher = smoother, slower).
    ///   - stepScale: Fraction of the raw dq step applied each iteration (0 < α ≤ 1).
    /// - Returns: An `IKResult` with the best joint angles found.
    public static func solve(
        model: RobotModel,
        endEffectorLinkName: String,
        targetTransform: simd_double4x4,
        initialAngles: [String: Double],
        maxIterations: Int = 150,
        tolerance: Double = 1e-4,
        dampingFactor: Double = 0.01,
        stepScale: Double = 0.5
    ) -> IKResult {

        // Ordered list of actuated joint names (BFS order, stable across iterations).
        let jointNames = model.orderedActuatedJointNames
        let n = jointNames.count
        guard n > 0 else {
            return IKResult(jointAngles: initialAngles, converged: false, iterations: 0, finalError: .infinity)
        }

        // Current configuration as a flat array (mirrors jointNames order).
        var q: [Double] = jointNames.map { initialAngles[$0] ?? 0.0 }

        // Joint limit arrays (parallel to q).
        let limits: [(Double, Double)] = jointNames.map { name in
            if let lim = model.joints[name]?.limits {
                return (lim.lower, lim.upper)
            }
            return (-.pi, .pi)
        }

        let lambda2 = dampingFactor * dampingFactor
        var finalErr = Double.infinity

        for iter in 0 ..< maxIterations {
            let angles = dict(names: jointNames, values: q)
            let fk = ForwardKinematics.compute(model: model, jointAngles: angles)
            let T_ee = fk.transform(for: endEffectorLinkName)

            // 6-vector error [Δpos (3), Δori (3)].
            let err6 = cartesianError(from: T_ee, to: targetTransform)
            finalErr = norm6(err6)

            if finalErr < tolerance {
                return IKResult(
                    jointAngles: angles,
                    converged: true,
                    iterations: iter,
                    finalError: finalErr
                )
            }

            // Build 6×n Jacobian via finite differences.
            // Each column j = (FK(q + δeⱼ).ee - FK(q).ee) / δ   (position part)
            //                  rotation difference / δ             (orientation part)
            let delta = 1e-6
            var J = [[Double]](repeating: [Double](repeating: 0, count: n), count: 6)

            for j in 0 ..< n {
                var qPlus = q
                qPlus[j] += delta
                let fkPlus = ForwardKinematics.compute(
                    model: model,
                    jointAngles: dict(names: jointNames, values: qPlus)
                )
                let T_plus = fkPlus.transform(for: endEffectorLinkName)
                let errPlus = cartesianError(from: T_ee, to: T_plus)
                // Column j of J ≈ (err(q+δeⱼ) - err(q)) / δ
                // Since err = target - current, the Jacobian column is (current(q+δ) - current(q)) / δ
                // which equals -errPlus/δ in our sign convention — easier: just use
                // the world-displacement approach directly.
                let dPos = T_plus.translation - T_ee.translation
                let dOri = orientationDelta(from: T_ee, to: T_plus)
                J[0][j] = dPos.x / delta
                J[1][j] = dPos.y / delta
                J[2][j] = dPos.z / delta
                J[3][j] = dOri.x / delta
                J[4][j] = dOri.y / delta
                J[5][j] = dOri.z / delta
            }

            // Damped pseudo-inverse step: dq = Jᵀ (J Jᵀ + λ²I)⁻¹ err
            // We compute (J Jᵀ + λ²I) as a 6×6 matrix, invert it, then
            // dq = Jᵀ · A⁻¹ · err   where A = JJᵀ + λ²I
            let A = add(matmul(J, transpose(J), rows: 6, inner: n, cols: 6),
                        identity6(scale: lambda2))
            guard let Ainv = invert6x6(A) else { continue }

            // v = A⁻¹ · err  (6-vector)
            let v = matvec6(Ainv, err6)

            // dq = Jᵀ · v  (n-vector)
            var dq = [Double](repeating: 0, count: n)
            for j in 0 ..< n {
                for i in 0 ..< 6 {
                    dq[j] += J[i][j] * v[i]
                }
            }

            // Apply step with scaling and clamp to joint limits.
            for j in 0 ..< n {
                q[j] = clamp(q[j] + stepScale * dq[j],
                             lo: limits[j].0, hi: limits[j].1)
            }
        }

        return IKResult(
            jointAngles: dict(names: jointNames, values: q),
            converged: false,
            iterations: maxIterations,
            finalError: finalErr
        )
    }

    // MARK: - Private helpers

    /// Build a dict from parallel name / value arrays.
    private static func dict(names: [String], values: [Double]) -> [String: Double] {
        var d = [String: Double](minimumCapacity: names.count)
        for (i, name) in names.enumerated() { d[name] = values[i] }
        return d
    }

    /// 6-vector Cartesian error [Δpos(3), Δori(3)] from current to target.
    ///
    /// Position error is straightforward. Orientation error is the rotation vector
    /// of  R_err = R_target · R_current⁻¹  extracted via the axis-angle formula.
    private static func cartesianError(
        from current: simd_double4x4,
        to target: simd_double4x4
    ) -> [Double] {
        let dp = target.translation - current.translation
        let dOri = orientationDelta(from: current, to: target)
        return [dp.x, dp.y, dp.z, dOri.x, dOri.y, dOri.z]
    }

    /// Rotation vector (axis × angle) representing R_target · R_current⁻¹.
    private static func orientationDelta(
        from current: simd_double4x4,
        to target: simd_double4x4
    ) -> SIMD3<Double> {
        // Extract 3×3 rotation submatrices.
        let Rc = upperLeft3x3(current)
        let Rt = upperLeft3x3(target)
        // R_err = Rt · Rc^T
        let Re = matmul3x3(Rt, transpose3x3(Rc))
        // Rotation vector from R_err: angle = acos((trace-1)/2), axis from skew part.
        let trace = Re[0][0] + Re[1][1] + Re[2][2]
        let cosA = ((trace - 1.0) / 2.0).clamped(to: -1...1)
        let angle = acos(cosA)
        if abs(angle) < 1e-9 { return .zero }
        let s = 1.0 / (2.0 * sin(angle))
        let ax = (Re[2][1] - Re[1][2]) * s
        let ay = (Re[0][2] - Re[2][0]) * s
        let az = (Re[1][0] - Re[0][1]) * s
        return SIMD3(ax, ay, az) * angle
    }

    private static func norm6(_ v: [Double]) -> Double {
        v.reduce(0) { $0 + $1 * $1 }.squareRoot()
    }

    private static func clamp(_ x: Double, lo: Double, hi: Double) -> Double {
        min(max(x, lo), hi)
    }

    // MARK: - Tiny matrix routines (avoids Accelerate dependency in the package)

    /// 6×6 identity scaled by `scale`.
    private static func identity6(scale: Double) -> [[Double]] {
        var m = [[Double]](repeating: [Double](repeating: 0, count: 6), count: 6)
        for i in 0 ..< 6 { m[i][i] = scale }
        return m
    }

    /// Matrix transpose for a rows×cols matrix stored as [[Double]] (row-major).
    private static func transpose(_ A: [[Double]]) -> [[Double]] {
        guard let first = A.first else { return [] }
        let rows = A.count, cols = first.count
        var R = [[Double]](repeating: [Double](repeating: 0, count: rows), count: cols)
        for i in 0 ..< rows { for j in 0 ..< cols { R[j][i] = A[i][j] } }
        return R
    }

    /// General matrix multiply A(rows×inner) · B(inner×cols) → rows×cols.
    private static func matmul(
        _ A: [[Double]], _ B: [[Double]],
        rows: Int, inner: Int, cols: Int
    ) -> [[Double]] {
        var C = [[Double]](repeating: [Double](repeating: 0, count: cols), count: rows)
        for i in 0 ..< rows {
            for k in 0 ..< inner {
                let aik = A[i][k]
                for j in 0 ..< cols { C[i][j] += aik * B[k][j] }
            }
        }
        return C
    }

    /// Element-wise add two matrices of the same shape.
    private static func add(_ A: [[Double]], _ B: [[Double]]) -> [[Double]] {
        A.enumerated().map { i, row in row.enumerated().map { j, v in v + B[i][j] } }
    }

    /// 6×6 matrix × 6-vector.
    private static func matvec6(_ M: [[Double]], _ v: [Double]) -> [Double] {
        (0 ..< 6).map { i in (0 ..< 6).reduce(0) { $0 + M[i][$1] * v[$1] } }
    }

    // MARK: - 3×3 helpers

    private static func upperLeft3x3(_ m: simd_double4x4) -> [[Double]] {
        [
            [m.columns.0.x, m.columns.1.x, m.columns.2.x],
            [m.columns.0.y, m.columns.1.y, m.columns.2.y],
            [m.columns.0.z, m.columns.1.z, m.columns.2.z]
        ]
    }

    private static func transpose3x3(_ A: [[Double]]) -> [[Double]] {
        [[A[0][0], A[1][0], A[2][0]],
         [A[0][1], A[1][1], A[2][1]],
         [A[0][2], A[1][2], A[2][2]]]
    }

    private static func matmul3x3(_ A: [[Double]], _ B: [[Double]]) -> [[Double]] {
        var C = [[Double]](repeating: [Double](repeating: 0, count: 3), count: 3)
        for i in 0 ..< 3 { for k in 0 ..< 3 { for j in 0 ..< 3 { C[i][j] += A[i][k] * B[k][j] } } }
        return C
    }

    // MARK: - 6×6 matrix inversion (Gauss-Jordan)

    private static func invert6x6(_ A: [[Double]]) -> [[Double]]? {
        var M = A          // working copy
        var I = identity6(scale: 1.0)
        let n = 6
        for col in 0 ..< n {
            // Find pivot.
            var pivotRow = col
            var pivotVal = abs(M[col][col])
            for row in (col + 1) ..< n {
                if abs(M[row][col]) > pivotVal {
                    pivotVal = abs(M[row][col])
                    pivotRow = row
                }
            }
            if pivotVal < 1e-12 { return nil }  // singular
            if pivotRow != col {
                M.swapAt(col, pivotRow)
                I.swapAt(col, pivotRow)
            }
            let inv = 1.0 / M[col][col]
            for j in 0 ..< n { M[col][j] *= inv; I[col][j] *= inv }
            for row in 0 ..< n where row != col {
                let f = M[row][col]
                for j in 0 ..< n { M[row][j] -= f * M[col][j]; I[row][j] -= f * I[col][j] }
            }
        }
        return I
    }
}

// MARK: - Comparable clamping helper

extension Comparable {
    fileprivate func clamped(to range: ClosedRange<Self>) -> Self {
        min(max(self, range.lowerBound), range.upperBound)
    }
}
