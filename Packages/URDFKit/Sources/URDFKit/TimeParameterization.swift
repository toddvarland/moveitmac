import Foundation

// MARK: - Output types

/// A single waypoint with timing and joint velocities.
public struct TimedWaypoint {
    /// Absolute time from the start of the trajectory (seconds).
    public let time: Double
    /// Joint angles at this waypoint.
    public let angles: [String: Double]
    /// Joint velocities at this waypoint (rad/s or m/s).
    public let velocities: [String: Double]
}

/// A time-parameterized trajectory produced by `TimeParameterization.apply(...)`.
public struct TimedTrajectory {
    public let waypoints: [TimedWaypoint]
    /// Total duration in seconds.
    public var duration: Double { waypoints.last?.time ?? 0 }
    /// Interpolate joint angles at an arbitrary time t ∈ [0, duration].
    public func angles(at t: Double) -> [String: Double] {
        guard waypoints.count > 1 else { return waypoints.first?.angles ?? [:] }
        let clamped = max(0, min(t, duration))
        // Binary search for the segment straddling t.
        var lo = 0, hi = waypoints.count - 2
        while lo < hi {
            let mid = (lo + hi) / 2
            if waypoints[mid + 1].time < clamped { lo = mid + 1 } else { hi = mid }
        }
        let a = waypoints[lo]
        let b = waypoints[lo + 1]
        let dt = b.time - a.time
        guard dt > 1e-12 else { return b.angles }
        let alpha = (clamped - a.time) / dt
        var result: [String: Double] = [:]
        for name in a.angles.keys {
            result[name] = (a.angles[name] ?? 0) + alpha * ((b.angles[name] ?? 0) - (a.angles[name] ?? 0))
        }
        return result
    }
}

// MARK: - TOTG (Trapezoidal Velocity Profile)

/// Time-Optimal Trajectory Generation using a per-joint trapezoidal velocity profile.
///
/// For each pair of consecutive waypoints the algorithm finds the most-constrained
/// joint — the one that, at its own velocity/acceleration limits, takes the longest to
/// reach the next waypoint — then scales all other joints down proportionally so every
/// joint arrives at exactly the same time (synchronised motion).
///
/// The algorithm is a pure Swift value-type function with no external dependencies.
public enum TimeParameterization {

    // MARK: - Public API

    /// Apply time parameterization to a geometric path.
    ///
    /// - Parameters:
    ///   - path:           Sequence of joint-angle configurations (≥ 2 waypoints).
    ///   - model:          Robot model (provides velocity limits from `JointLimits`).
    ///   - maxVelocity:    Override max velocity (rad/s) applied to all joints.
    ///                     If nil, per-joint `JointLimits.velocity` is used (or 1.0 as fallback).
    ///   - maxAcceleration:Override max acceleration (rad/s²) for all joints.
    ///                     If nil, defaults to `2 × maxVelocity` per joint.
    /// - Returns: A `TimedTrajectory` whose duration reflects realistic joint kinematics.
    public static func apply(
        path:             [[String: Double]],
        model:            RobotModel,
        maxVelocity:      Double? = nil,
        maxAcceleration:  Double? = nil
    ) -> TimedTrajectory {
        guard path.count >= 2 else {
            let wps: [TimedWaypoint] = path.first.map {
                [TimedWaypoint(time: 0, angles: $0, velocities: [:])]
            } ?? []
            return TimedTrajectory(waypoints: wps)
        }

        let joints = model.orderedActuatedJointNames

        // Build per-joint limits.
        // Note: URDF convention allows velocity="0" to mean "unspecified/unlimited".
        // Treat zero (or missing) limits as 1.0 rad/s to avoid astronomically long durations.
        let vMax: [String: Double] = Dictionary(uniqueKeysWithValues: joints.map { name in
            let urdfV = model.joints[name]?.limits?.velocity ?? 0
            let v = maxVelocity ?? (urdfV > 0 ? urdfV : 1.0)
            return (name, max(v, 1e-9))
        })
        let aMax: [String: Double] = Dictionary(uniqueKeysWithValues: joints.map { name in
            let a = maxAcceleration ?? (vMax[name]! * 2.0)
            return (name, max(a, 1e-9))
        })

        // ── Pass 1: forward pass — assign times and velocities ────────────

        // Each segment: find the minimum time such that every joint can move
        // its required Δq while obeying its v/a limits.  We use the trapezoidal
        // formula: given Δq, v_max, a_max, the minimum time assuming zero
        // entry/exit velocity is:
        //   If Δq ≤ v²/(2a)  → triangular: t = 2√(Δq/a)
        //   Else              → trapezoidal: t = Δq/v + v/a
        // We pick the most constrained (longest) joint for this segment.

        struct Segment {
            let dt: Double          // duration
            let deltas: [String: Double]
            let vEntry: [String: Double]
            let vExit:  [String: Double]
        }

        var times: [Double] = [0]
        var segVEntry: [[String: Double]] = [zeroVelocity(joints)]
        var segVExit:  [[String: Double]] = []

        for i in 0 ..< path.count - 1 {
            let from = path[i], to = path[i + 1]
            var segTime = 0.0
            for name in joints {
                let dq = abs((to[name] ?? 0) - (from[name] ?? 0))
                let v  = vMax[name]!
                let a  = aMax[name]!
                let t  = trapTime(dq: dq, vMax: v, aMax: a)
                segTime = max(segTime, t)
            }
            segTime = max(segTime, 1e-6)  // avoid zero-duration segments

            // Compute per-joint peak velocity for this segment given segTime.
            var vExit: [String: Double] = [:]
            for name in joints {
                let dq = abs((to[name] ?? 0) - (from[name] ?? 0))
                // For synchronised motion peak v ≈ dq / (segTime - v/a)
                // but clamp to vMax.
                let v  = min(dq / segTime * 1.5, vMax[name]!)
                vExit[name] = (to[name] ?? 0) >= (from[name] ?? 0) ? v : -v
            }

            times.append(times.last! + segTime)
            segVExit.append(vExit)
            segVEntry.append(vExit)   // entry of next = exit of current (will be blended below)
        }
        // Last waypoint has zero velocity (trajectory comes to rest).
        segVEntry[segVEntry.count - 1] = zeroVelocity(joints)
        segVExit.append(zeroVelocity(joints))

        // ── Assemble waypoints ─────────────────────────────────────────────
        var waypoints: [TimedWaypoint] = []
        for i in 0 ..< path.count {
            let vel: [String: Double]
            if i == 0 {
                vel = zeroVelocity(joints)
            } else if i == path.count - 1 {
                vel = zeroVelocity(joints)
            } else {
                // Average entry/exit velocities at interior point for a smooth profile.
                var avg: [String: Double] = [:]
                for name in joints {
                    avg[name] = ((segVExit[i - 1][name] ?? 0) + (segVEntry[i][name] ?? 0)) / 2
                }
                vel = avg
            }
            waypoints.append(TimedWaypoint(time: times[i], angles: path[i], velocities: vel))
        }

        return TimedTrajectory(waypoints: waypoints)
    }

    // MARK: - Helpers

    /// Minimum time for a trapezoidal/triangular profile to move `dq` from rest.
    private static func trapTime(dq: Double, vMax: Double, aMax: Double) -> Double {
        guard dq > 1e-12 else { return 0 }
        // Distance covered during ramp-up + ramp-down at vMax: d_ramp = v²/a
        let dRamp = vMax * vMax / aMax
        if dq <= dRamp {
            // Triangular profile — never reaches vMax.
            return 2 * sqrt(dq / aMax)
        } else {
            // Trapezoidal profile.
            return dq / vMax + vMax / aMax
        }
    }

    private static func zeroVelocity(_ joints: [String]) -> [String: Double] {
        Dictionary(uniqueKeysWithValues: joints.map { ($0, 0.0) })
    }
}
