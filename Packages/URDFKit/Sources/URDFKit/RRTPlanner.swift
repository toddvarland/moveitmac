import Foundation

// MARK: - Internal tree node

private struct RRTNode {
    var config: [String: Double]
    var parent: Int  // -1 for root
}

// MARK: - RRT-Connect planner

/// Bidirectional RRT-Connect motion planner operating in joint space.
///
/// The planner is a pure function with no side effects beyond the returned
/// `Result` value.  It is safe to call from any thread.
public enum RRTPlanner {

    // MARK: - Public types

    public struct Result {
        /// Sequence of joint-angle configurations from start to goal.
        /// Empty on failure.
        public let path:       [[String: Double]]
        /// Number of grow iterations executed.
        public let iterations: Int
        /// Whether a path was found within `maxIterations`.
        public let success:    Bool
    }

    // MARK: - Public API

    /// Plan a collision-free joint-space path from `start` to `goal`.
    ///
    /// - Parameters:
    ///   - model:             Robot model (joint names and limits only).
    ///   - start:             Start configuration.
    ///   - goal:              Goal configuration.
    ///   - isCollisionFree:   Closure returning `true` iff a config is valid.
    ///   - maxIterations:     RRT grow iterations (default 5 000).
    ///   - stepSize:          Max normalised step per iteration (default 0.1 ≈ 10% of range).
    ///   - goalBias:          Probability of sampling the goal (default 0.05).
    ///   - connectTolerance:  Normalised distance threshold for tree connection (default 0.05).
    public static func plan(
        model:             RobotModel,
        start:             [String: Double],
        goal:              [String: Double],
        isCollisionFree:   ([String: Double]) -> Bool,
        maxIterations:     Int    = 5_000,
        stepSize:          Double = 0.1,
        goalBias:          Double = 0.05,
        connectTolerance:  Double = 0.05
    ) -> Result {

        let joints = model.orderedActuatedJointNames
        guard !joints.isEmpty else {
            return Result(path: [], iterations: 0, success: false)
        }

        // Per-joint normalisation range (fallback: 2π for unlimited joints).
        let ranges: [Double] = joints.map { name in
            guard let j = model.joints[name], let lim = j.limits else { return 2 * .pi }
            return max(lim.upper - lim.lower, 1e-9)
        }

        // Fast-fail if start or goal are themselves in collision.
        guard isCollisionFree(start), isCollisionFree(goal) else {
            return Result(path: [], iterations: 0, success: false)
        }

        // Already close enough?
        if dist(start, goal, joints: joints, ranges: ranges) < connectTolerance {
            let smoothed = smooth([start, goal],
                                  joints: joints, ranges: ranges,
                                  isCollisionFree: isCollisionFree)
            return Result(path: smoothed, iterations: 0, success: true)
        }

        // ── Two trees growing toward each other ───────────────────────────
        var treeS: [RRTNode] = [RRTNode(config: start, parent: -1)]
        var treeG: [RRTNode] = [RRTNode(config: goal,  parent: -1)]
        var startTreeIsS = true   // which tree has 'start' as its root

        var rng = SystemRandomNumberGenerator()

        for iter in 0 ..< maxIterations {

            // ── 1. Sample ─────────────────────────────────────────────────
            let qRand: [String: Double] = Double.random(in: 0...1, using: &rng) < goalBias
                ? treeG[0].config
                : randomConfig(joints: joints, model: model, rng: &rng)

            // ── 2. Extend treeS one step toward qRand ─────────────────────
            guard let newIdxS = extendTree(&treeS, toward: qRand,
                                           joints: joints, ranges: ranges,
                                           stepSize: stepSize,
                                           isCollisionFree: isCollisionFree)
            else {
                // Trapped — swap anyway so next iteration starts from other side.
                swap(&treeS, &treeG)
                startTreeIsS.toggle()
                continue
            }

            // ── 3. Greedily connect treeG toward the new treeS node ───────
            if let newIdxG = connectTree(&treeG, toward: treeS[newIdxS].config,
                                         joints: joints, ranges: ranges,
                                         stepSize: stepSize, tolerance: connectTolerance,
                                         isCollisionFree: isCollisionFree)
            {
                // Build start → goal path from the two tree branches.
                let pathS = extractPath(treeS, from: newIdxS)  // root(S) → newIdxS
                let pathG = extractPath(treeG, from: newIdxG)  // root(G) → newIdxG

                // Assemble so the result runs start → goal regardless of swap parity.
                let raw = startTreeIsS
                    ? pathS + pathG.reversed()
                    : pathG + pathS.reversed()

                let smoothed = smooth(raw,
                                      joints: joints, ranges: ranges,
                                      isCollisionFree: isCollisionFree)
                return Result(path: smoothed, iterations: iter + 1, success: true)
            }

            // ── 4. Swap trees (RRT-Connect bidirectional alternation) ─────
            swap(&treeS, &treeG)
            startTreeIsS.toggle()
        }

        return Result(path: [], iterations: maxIterations, success: false)
    }

    // MARK: - Tree operations

    /// Extend `tree` one step toward `target`.  Returns new node index or nil if trapped.
    @discardableResult
    private static func extendTree(
        _ tree:          inout [RRTNode],
        toward target:   [String: Double],
        joints:          [String],
        ranges:          [Double],
        stepSize:        Double,
        isCollisionFree: ([String: Double]) -> Bool
    ) -> Int? {
        let nearIdx  = nearestIndex(in: tree, to: target, joints: joints, ranges: ranges)
        let newConfig = steer(from: tree[nearIdx].config, toward: target,
                              joints: joints, ranges: ranges, stepSize: stepSize)
        guard isCollisionFree(newConfig) else { return nil }
        tree.append(RRTNode(config: newConfig, parent: nearIdx))
        return tree.count - 1
    }

    /// Greedily extend `tree` toward `target` until within `tolerance` or trapped.
    /// Returns the index of the arrival node if the tree reached within `tolerance`.
    private static func connectTree(
        _ tree:          inout [RRTNode],
        toward target:   [String: Double],
        joints:          [String],
        ranges:          [Double],
        stepSize:        Double,
        tolerance:       Double,
        isCollisionFree: ([String: Double]) -> Bool
    ) -> Int? {
        // Check if the current nearest node is already close enough.
        let nearIdx = nearestIndex(in: tree, to: target, joints: joints, ranges: ranges)
        if dist(tree[nearIdx].config, target, joints: joints, ranges: ranges) < tolerance {
            return nearIdx
        }
        // Step until within tolerance or blocked.
        while true {
            guard let newIdx = extendTree(&tree, toward: target,
                                          joints: joints, ranges: ranges,
                                          stepSize: stepSize,
                                          isCollisionFree: isCollisionFree)
            else { return nil }   // TRAPPED

            if dist(tree[newIdx].config, target, joints: joints, ranges: ranges) < tolerance {
                return newIdx   // REACHED
            }
            // ADVANCED — keep going
        }
    }

    // MARK: - Path utilities

    /// Walk parent pointers from `leafIdx` to root; returns root-first path.
    private static func extractPath(_ tree: [RRTNode], from leafIdx: Int) -> [[String: Double]] {
        var path: [[String: Double]] = []
        var idx = leafIdx
        while idx >= 0 {
            path.append(tree[idx].config)
            idx = tree[idx].parent
        }
        return path.reversed()
    }

    /// Greedy shortcutter: repeatedly try to skip intermediate waypoints while
    /// the direct segment remains collision-free.
    private static func smooth(
        _ path:          [[String: Double]],
        joints:          [String],
        ranges:          [Double],
        isCollisionFree: ([String: Double]) -> Bool,
        checkSteps:      Int = 10
    ) -> [[String: Double]] {
        guard path.count > 2 else { return path }
        var result = path
        var improved = true
        while improved {
            improved = false
            var i = 0
            while i + 2 < result.count {
                if isDirectSegmentFree(result[i], result[i + 2],
                                       joints: joints,
                                       isCollisionFree: isCollisionFree,
                                       checkSteps: checkSteps)
                {
                    result.remove(at: i + 1)
                    improved = true
                    // Don't advance i — try to skip the next node from same position.
                } else {
                    i += 1
                }
            }
        }
        return result
    }

    /// Returns true iff the straight-line segment from `a` to `b` in joint
    /// space is entirely collision-free (sampled at `checkSteps` equal intervals).
    private static func isDirectSegmentFree(
        _ a: [String: Double], _ b: [String: Double],
        joints:          [String],
        isCollisionFree: ([String: Double]) -> Bool,
        checkSteps:      Int
    ) -> Bool {
        for k in 1 ... checkSteps {
            let t = Double(k) / Double(checkSteps)
            var q: [String: Double] = [:]
            for name in joints {
                q[name] = (a[name] ?? 0) + t * ((b[name] ?? 0) - (a[name] ?? 0))
            }
            if !isCollisionFree(q) { return false }
        }
        return true
    }

    // MARK: - Geometric helpers

    /// Normalised L2 distance in joint space.
    private static func dist(
        _ a: [String: Double], _ b: [String: Double],
        joints: [String], ranges: [Double]
    ) -> Double {
        var sum = 0.0
        for (i, name) in joints.enumerated() {
            let d = ((a[name] ?? 0) - (b[name] ?? 0)) / ranges[i]
            sum += d * d
        }
        return sqrt(sum)
    }

    /// Index of the node in `tree` closest to `target` in normalised joint space.
    private static func nearestIndex(
        in tree: [RRTNode], to target: [String: Double],
        joints: [String], ranges: [Double]
    ) -> Int {
        var bestIdx  = 0
        var bestDist = Double.infinity
        for (i, node) in tree.enumerated() {
            let d = dist(node.config, target, joints: joints, ranges: ranges)
            if d < bestDist { bestDist = d; bestIdx = i }
        }
        return bestIdx
    }

    /// Step `stepSize` (normalised) from `a` toward `b`; returns `b` if already closer.
    private static func steer(
        from a: [String: Double], toward b: [String: Double],
        joints: [String], ranges: [Double], stepSize: Double
    ) -> [String: Double] {
        let d = dist(a, b, joints: joints, ranges: ranges)
        if d <= stepSize { return b }
        let t = stepSize / d
        var result = a
        for name in joints {
            result[name] = (a[name] ?? 0) + t * ((b[name] ?? 0) - (a[name] ?? 0))
        }
        return result
    }

    /// Uniform random configuration within joint limits.
    private static func randomConfig(
        joints: [String], model: RobotModel,
        rng:    inout SystemRandomNumberGenerator
    ) -> [String: Double] {
        var config: [String: Double] = [:]
        for name in joints {
            if let j = model.joints[name], let lim = j.limits {
                config[name] = Double.random(in: lim.lower ... lim.upper, using: &rng)
            } else {
                config[name] = Double.random(in: -.pi ... .pi, using: &rng)
            }
        }
        return config
    }
}
