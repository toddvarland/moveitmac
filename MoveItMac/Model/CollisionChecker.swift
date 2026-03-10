import Foundation
import simd
import URDFKit

// MARK: - Result types

struct CollisionPair: Equatable, Hashable {
    let linkA: String
    let linkB: String

    /// Canonical order so CollisionPair("A","B") == CollisionPair("B","A").
    init(_ a: String, _ b: String) {
        if a <= b { linkA = a; linkB = b } else { linkA = b; linkB = a }
    }
}

struct ObstacleHit: Equatable {
    let linkName: String
    let obstacleID: UUID
}

struct CollisionResult: Equatable {
    let collidingLinks: Set<String>
    let selfCollisions: [CollisionPair]
    let obstacleHits:   [ObstacleHit]

    var hasCollision: Bool { !collidingLinks.isEmpty }

    static let clear = CollisionResult(collidingLinks: [], selfCollisions: [], obstacleHits: [])

    // Compare only the set of colliding links — cheap equality used to guard
    // against 120 Hz SwiftUI re-renders when nothing has changed.
    static func == (lhs: CollisionResult, rhs: CollisionResult) -> Bool {
        lhs.collidingLinks == rhs.collidingLinks
    }
}

// MARK: - Checker

/// Bounding-sphere collision checker.
/// All inputs are value types; safe to call on any thread.
enum CollisionChecker {

    static func check(
        model: RobotModel,
        fkResult: FKResult,
        obstacles: [Obstacle]
    ) -> CollisionResult {

        // Build world-space bounding sphere for every link.
        struct Sphere {
            let name: String
            let center: SIMD3<Double>
            let radius: Double
        }

        let spheres: [Sphere] = model.links.map { (name, link) in
            let (localCenter, radius) = boundingBall(link: link)
            let worldT = fkResult.transform(for: name)
            // Apply the link world transform to the visual/collision center offset.
            let h = worldT * SIMD4<Double>(localCenter.x, localCenter.y, localCenter.z, 1)
            return Sphere(name: name, center: SIMD3<Double>(h.x, h.y, h.z), radius: radius)
        }

        let adjacent = adjacentPairs(model)
        var selfCols: [CollisionPair] = []
        var obsHits:  [ObstacleHit]  = []
        var hitting   = Set<String>()

        // Self-collision: O(n²) sphere-sphere, skip directly-connected links.
        for i in 0 ..< spheres.count {
            for j in (i + 1) ..< spheres.count {
                let a = spheres[i]; let b = spheres[j]
                guard !adjacent.contains(CollisionPair(a.name, b.name)) else { continue }
                if simd_length(a.center - b.center) < a.radius + b.radius {
                    selfCols.append(CollisionPair(a.name, b.name))
                    hitting.insert(a.name); hitting.insert(b.name)
                }
            }
        }

        // Obstacle collisions.
        for sphere in spheres {
            for obstacle in obstacles {
                let (obsCenter, obsRadius) = obstacleBall(obstacle)
                if simd_length(sphere.center - obsCenter) < sphere.radius + obsRadius {
                    obsHits.append(ObstacleHit(linkName: sphere.name, obstacleID: obstacle.id))
                    hitting.insert(sphere.name)
                }
            }
        }

        return CollisionResult(collidingLinks: hitting, selfCollisions: selfCols, obstacleHits: obsHits)
    }

    // MARK: - Private helpers

    private static func boundingBall(link: Link) -> (SIMD3<Double>, Double) {
        // Prefer <collision> geometry; fall back to <visual>.
        let geom   = link.collision?.geometry ?? link.visual?.geometry
        let center: SIMD3<Double> = link.collision?.origin.xyz
                                 ?? link.visual?.origin.xyz
                                 ?? .zero
        let radius: Double
        switch geom {
        case .sphere(let r):            radius = r
        case .cylinder(let r, let h):  radius = (r * r + (h / 2) * (h / 2)).squareRoot()
        case .box(let s):              radius = simd_length(s) / 2
        case .mesh(_, let scale):      radius = simd_length(scale) * 0.15
        case nil:                      radius = 0.03
        }
        return (center, max(radius, 0.01))
    }

    private static func obstacleBall(_ obs: Obstacle) -> (SIMD3<Double>, Double) {
        let r: Double
        switch obs.shape {
        case .sphere:   r = obs.radius
        case .cylinder: r = (obs.radius * obs.radius + (obs.height / 2) * (obs.height / 2)).squareRoot()
        case .box:      r = simd_length(obs.boxSize) / 2
        }
        return (obs.position, r)
    }

    /// Set of directly-adjacent link pairs that share a joint — never checked for self-collision.
    private static func adjacentPairs(_ model: RobotModel) -> Set<CollisionPair> {
        model.joints.values.reduce(into: Set<CollisionPair>()) { set, joint in
            set.insert(CollisionPair(joint.parentLinkName, joint.childLinkName))
        }
    }
}
